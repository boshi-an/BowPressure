#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int) { g_running = 0; }

static uint16_t modbus_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

static int serial_open(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        std::perror("open serial port");
        return -1;
    }

    // Clear non-blocking after open so reads can block with timeout
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::perror("tcgetattr");
        close(fd);
        return -1;
    }

    // 38400 8N1; baud rate is ignored over USB-CDC
    cfsetispeed(&tty, B19200);
    cfsetospeed(&tty, B19200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT |
                      PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Inter-character timeout 100ms, minimum 0 bytes
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

// Build a Modbus RTU "Read Holding Registers" (FC 03) request.
// Reads `count` registers starting at `start_reg` from device `addr`.
static size_t build_read_request(uint8_t *buf, uint8_t addr,
                                 uint16_t start_reg, uint16_t count) {
    buf[0] = addr;
    buf[1] = 0x03;
    buf[2] = static_cast<uint8_t>(start_reg >> 8);
    buf[3] = static_cast<uint8_t>(start_reg & 0xFF);
    buf[4] = static_cast<uint8_t>(count >> 8);
    buf[5] = static_cast<uint8_t>(count & 0xFF);
    uint16_t crc = modbus_crc16(buf, 6);
    buf[6] = static_cast<uint8_t>(crc & 0xFF);
    buf[7] = static_cast<uint8_t>(crc >> 8);
    return 8;
}

// Build a Modbus RTU "Write Single Register" (FC 06) request.
static size_t build_write_request(uint8_t *buf, uint8_t addr,
                                  uint16_t reg, uint16_t value) {
    buf[0] = addr;
    buf[1] = 0x06;
    buf[2] = static_cast<uint8_t>(reg >> 8);
    buf[3] = static_cast<uint8_t>(reg & 0xFF);
    buf[4] = static_cast<uint8_t>(value >> 8);
    buf[5] = static_cast<uint8_t>(value & 0xFF);
    uint16_t crc = modbus_crc16(buf, 6);
    buf[6] = static_cast<uint8_t>(crc & 0xFF);
    buf[7] = static_cast<uint8_t>(crc >> 8);
    return 8;
}

// Read exactly `need` bytes from fd, with a total timeout in milliseconds.
// Returns number of bytes actually read.
static size_t serial_read(int fd, uint8_t *buf, size_t need, int timeout_ms) {
    size_t got = 0;
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (got < need) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) break;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        struct timeval tv;
        tv.tv_sec = remaining.count() / 1000;
        tv.tv_usec = (remaining.count() % 1000) * 1000;

        int sel = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (sel <= 0) break;

        ssize_t n = read(fd, buf + got, need - got);
        if (n <= 0) break;
        got += static_cast<size_t>(n);
    }
    return got;
}

// Send a FC06 write and wait for the echo response. Returns true on success.
static bool write_register(int fd, uint8_t addr, uint16_t reg, uint16_t value) {
    uint8_t buf[8];
    size_t len = build_write_request(buf, addr, reg, value);
    tcflush(fd, TCIOFLUSH);
    if (write(fd, buf, len) != static_cast<ssize_t>(len)) return false;
    uint8_t resp[8];
    size_t n = serial_read(fd, resp, 8, 300);
    if (n < 8) return false;
    uint16_t crc_recv = static_cast<uint16_t>(resp[6]) |
                        (static_cast<uint16_t>(resp[7]) << 8);
    return crc_recv == modbus_crc16(resp, 6);
}

// Parse a Modbus FC03 response for 2 registers into a signed 32-bit value.
// Register 0 = low word, Register 1 = high word (per CMCU-06A spec).
// Returns true on success.
static bool parse_response(const uint8_t *buf, size_t len, int32_t &value) {
    // Expected: addr(1) + fc(1) + bytecount(1) + data(4) + crc(2) = 9
    if (len < 9) return false;
    if (buf[1] != 0x03) return false;
    if (buf[2] != 0x04) return false;

    uint16_t crc_recv =
        static_cast<uint16_t>(buf[len - 2]) |
        (static_cast<uint16_t>(buf[len - 1]) << 8);
    uint16_t crc_calc = modbus_crc16(buf, len - 2);
    if (crc_recv != crc_calc) return false;

    uint16_t reg0 = (static_cast<uint16_t>(buf[3]) << 8) | buf[4];
    uint16_t reg1 = (static_cast<uint16_t>(buf[5]) << 8) | buf[6];
    value = static_cast<int32_t>((static_cast<uint32_t>(reg1) << 16) | reg0);
    return true;
}

static std::string timestamp_now() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) %
              1000;
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

int main(int argc, char *argv[]) {
    const char *port = "/dev/ttyACM0";
    double hz = 10.0;
    const char *logfile = "bowpressure_log.csv";

    if (argc > 1) port = argv[1];
    if (argc > 2) {
        hz = std::atof(argv[2]);
        if (hz <= 0.0) {
            std::cerr << "Error: frame rate must be positive\n";
            return 1;
        }
    }
    if (argc > 3) logfile = argv[3];

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    int fd = serial_open(port);
    if (fd < 0) return 1;

    // Configure CMCU-06A: increase internal sampling rate
    std::cout << "Configuring CMCU-06A...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    constexpr uint8_t ADDR = 0x01;
    // Disable write protection (register 23 = 1)
    if (!write_register(fd, ADDR, 23, 1))
        std::cerr << "[warn] failed to disable write protection\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set sampling rate to 640 Hz (register 14: 1=10Hz 2=40Hz 3=640Hz 4=1280Hz)
    if (!write_register(fd, ADDR, 14, 2))
        std::cerr << "[warn] failed to set sampling rate\n";
    else
        std::cout << "  Sampling rate -> 40 Hz\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Set average filter to 50 (register 17, recommended 50-100 for 640 Hz)
    if (!write_register(fd, ADDR, 17, 1))
        std::cerr << "[warn] failed to set average filter\n";
    else
        std::cout << "  Average filter -> 1\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Re-enable write protection (register 23 = 0)
    write_register(fd, ADDR, 23, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::ofstream csv(logfile, std::ios::trunc);
    if (!csv.is_open()) {
        std::cerr << "Error: cannot open log file " << logfile << "\n";
        close(fd);
        return 1;
    }
    csv << "timestamp,raw_value\n";

    std::cout << "Reading from " << port << " at " << hz << " Hz\n"
              << "Logging to " << logfile << "\n"
              << "Press Ctrl+C to stop.\n\n";

    const auto interval = std::chrono::microseconds(
        static_cast<int64_t>(1'000'000.0 / hz));

    // Modbus RTU request: read 2 registers starting at address 0 from device 1
    uint8_t req[8];
    size_t req_len = build_read_request(req, 0x01, 0x0000, 0x0002);

    uint64_t sample_count = 0;
    uint64_t error_count = 0;
    uint64_t fps_frame_count = 0;
    auto fps_timer = std::chrono::steady_clock::now();
    double current_fps = 0.0;

    while (g_running) {
        auto t_start = std::chrono::steady_clock::now();

        tcflush(fd, TCIFLUSH);

        ssize_t written = write(fd, req, req_len);
        if (written != static_cast<ssize_t>(req_len)) {
            std::cerr << "[warn] write failed\n";
            ++error_count;
            std::this_thread::sleep_until(t_start + interval);
            continue;
        }

        uint8_t resp[32];
        size_t n = serial_read(fd, resp, 9, 200);

        int32_t value = 0;
        if (parse_response(resp, n, value)) {
            ++sample_count;
            ++fps_frame_count;

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - fps_timer).count();
            if (dt >= 1.0) {
                current_fps = fps_frame_count / dt;
                fps_frame_count = 0;
                fps_timer = now;
            }

            std::string ts = timestamp_now();
            std::cout << "\r[" << ts << "]  value: " << std::setw(10) << value
                      << "  fps: " << std::fixed << std::setprecision(1)
                      << std::setw(5) << current_fps
                      << "  (samples: " << sample_count
                      << ", errors: " << error_count << ")" << std::flush;
            csv << ts << "," << value << "\n";
            csv.flush();
        } else {
            ++error_count;
            std::cerr << "\r[warn] bad response (" << n << " bytes)";
            if (n > 0) {
                std::cerr << ":";
                for (size_t i = 0; i < n; ++i)
                    std::fprintf(stderr, " %02X", resp[i]);
            }
            std::cerr << std::flush;
        }

        auto t_end = std::chrono::steady_clock::now();
        auto elapsed = t_end - t_start;
        if (elapsed < interval)
            std::this_thread::sleep_for(interval - elapsed);
    }

    std::cout << "\n\nStopped. " << sample_count << " samples, "
              << error_count << " errors.\n";

    close(fd);
    csv.close();
    return 0;
}
