#include <iostream>
#include <cstring>
#include <cstdint>
#include <csignal>
#include <vector>
#include <fstream>
#include <alsa/asoundlib.h>
#include <ltc.h>

// --- Configuration ---
static const char*   ALSA_DEVICE  = "hw:0,0";   // change to your device
static const double  SAMPLE_RATE  = 48000.0;
static const int     FPS          = 25;
static const int     CHANNELS     = 1;
static const int     BIT_DEPTH    = 16;          // S16_LE
static const int     PERIOD_SIZE  = 1920;        // samples per period

static volatile bool g_running = true;
void on_signal(int) { g_running = false; }

static void write_le16(std::ofstream& os, uint16_t v) {
    char b[2];
    b[0] = (char)(v & 0xff);
    b[1] = (char)((v >> 8) & 0xff);
    os.write(b, 2);
}

static void write_le32(std::ofstream& os, uint32_t v) {
    char b[4];
    b[0] = (char)(v & 0xff);
    b[1] = (char)((v >> 8) & 0xff);
    b[2] = (char)((v >> 16) & 0xff);
    b[3] = (char)((v >> 24) & 0xff);
    os.write(b, 4);
}

static bool write_wav_header(std::ofstream& os, uint32_t sample_rate,
                             uint16_t channels, uint16_t bits_per_sample,
                             uint32_t data_bytes)
{
    if (!os) return false;
    const uint32_t byte_rate = sample_rate * channels * bits_per_sample / 8;
    const uint16_t block_align = (uint16_t)(channels * bits_per_sample / 8);
    const uint32_t riff_size = 36u + data_bytes;

    os.seekp(0, std::ios::beg);
    os.write("RIFF", 4);
    write_le32(os, riff_size);
    os.write("WAVE", 4);
    os.write("fmt ", 4);
    write_le32(os, 16);               // PCM fmt chunk size
    write_le16(os, 1);                // PCM format
    write_le16(os, channels);
    write_le32(os, sample_rate);
    write_le32(os, byte_rate);
    write_le16(os, block_align);
    write_le16(os, bits_per_sample);
    os.write("data", 4);
    write_le32(os, data_bytes);
    return (bool)os;
}

// ------------------------------------------------------------------ ALSA init
snd_pcm_t* open_alsa(const char* device, unsigned int rate,
                     int channels, snd_pcm_uframes_t period)
{
    snd_pcm_t*          pcm  = nullptr;
    snd_pcm_hw_params_t* hw  = nullptr;

    if (snd_pcm_open(&pcm, device, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        std::cerr << "Cannot open device: " << device << "\n";
        return nullptr;
    }

    snd_pcm_hw_params_alloca(&hw);
    snd_pcm_hw_params_any(pcm, hw);
    snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm, hw, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm, hw, channels);
    snd_pcm_hw_params_set_rate(pcm, hw, rate, 0);
    snd_pcm_hw_params_set_period_size(pcm, hw, period, 0);

    if (snd_pcm_hw_params(pcm, hw) < 0) {
        std::cerr << "Cannot set HW params\n";
        snd_pcm_close(pcm);
        return nullptr;
    }
    snd_pcm_prepare(pcm);
    return pcm;
}

// ------------------------------------------------------------------ main
int main(int argc, char* argv[])
{
    // Optional args:
    //   argv[1] = ALSA device (e.g. "hw:1,0")
    //   argv[2] = optional .wav output path
    const char* device = (argc > 1) ? argv[1] : ALSA_DEVICE;
    const char* wav_path = (argc > 2) ? argv[2] : nullptr;

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ---- Open ALSA ----
    snd_pcm_t* pcm = open_alsa(device, (unsigned)SAMPLE_RATE, CHANNELS, PERIOD_SIZE);
    if (!pcm) return 1;
    std::cout << "Opened ALSA device: " << device << "\n";

    // ---- Create LTC encoder ----
    // LTC_TV_625_50 = PAL (25fps).  Use LTC_TV_525_60 for 29.97/30fps.
    LTCEncoder* enc = ltc_encoder_create(
        SAMPLE_RATE,
        FPS,
        (FPS == 25) ? LTC_TV_625_50 : LTC_TV_525_60,
        LTC_USE_DATE          // embed date in user bits
    );
    if (!enc) { std::cerr << "Cannot create LTC encoder\n"; return 1; }

    // ---- Starting timecode  (HH:MM:SS:FF) ----
    SMPTETimecode tc{};
    tc.hours   = 0;
    tc.mins    = 0;
    tc.secs    = 0;
    tc.frame   = 0;
    // Optional: fill date fields if using LTC_USE_DATE
    tc.years   = 25;   // 2025
    tc.months  = 4;
    tc.days    = 7;

    ltc_encoder_set_timecode(enc, &tc);

    // ---- Playback buffer ----
    // libltc produces one frame worth of samples per encode call.
    // SAMPLE_RATE/FPS = samples per frame (e.g. 48000/25 = 1920).
    const int SAMPLES_PER_FRAME = (int)(SAMPLE_RATE / FPS);
    std::vector<int16_t> pcm_buf(SAMPLES_PER_FRAME);

    std::ofstream wav;
    uint64_t wav_data_bytes = 0;
    if (wav_path) {
        wav.open(wav_path, std::ios::binary | std::ios::trunc);
        if (!wav || !write_wav_header(wav, (uint32_t)SAMPLE_RATE, (uint16_t)CHANNELS,
                                      (uint16_t)BIT_DEPTH, 0)) {
            std::cerr << "Cannot open/write WAV file: " << wav_path << "\n";
            snd_pcm_close(pcm);
            ltc_encoder_free(enc);
            return 1;
        }
        std::cout << "Saving LTC audio to: " << wav_path << "\n";
    }

    std::cout << "Streaming LTC at " << FPS << " fps  (Ctrl-C to stop)\n";

    while (g_running) {
        // Encode one LTC frame → raw ltcsnd_sample_t bytes
        ltc_encoder_encode_frame(enc);

        // Retrieve encoded audio via current non-deprecated API
        int raw_len = (int)ltc_encoder_get_buffersize(enc);
        ltcsnd_sample_t* raw = nullptr;
        ltc_encoder_get_bufferptr(enc, &raw, 1);  // 1 = flush after read

        if (!raw || raw_len <= 0) {
            ltc_encoder_inc_timecode(enc);
            continue;
        }

        // ltcsnd_sample_t is unsigned char (0–255, centre = 128).
        // Scale to S16_LE  (-32768 … +32767).
        int n = std::min(raw_len, SAMPLES_PER_FRAME);
        for (int i = 0; i < n; ++i)
            pcm_buf[i] = ((int16_t)raw[i] - 128) * 256;

        // Optional file save: write generated S16_LE PCM to WAV data section.
        if (wav.is_open()) {
            const std::streamsize bytes = (std::streamsize)(n * (int)sizeof(int16_t));
            wav.write(reinterpret_cast<const char*>(pcm_buf.data()), bytes);
            wav_data_bytes += (uint64_t)bytes;
            if (!wav) {
                std::cerr << "WAV write error\n";
                g_running = false;
            }
        }

        // Write to ALSA (handles short writes / XRUN recovery)
        int written = 0;
        while (written < n && g_running) {
            int rc = snd_pcm_writei(pcm, pcm_buf.data() + written, n - written);
            if (rc == -EPIPE) {                     // underrun
                snd_pcm_prepare(pcm);
            } else if (rc < 0) {
                std::cerr << "ALSA write error: " << snd_strerror(rc) << "\n";
                g_running = false;
            } else {
                written += rc;
            }
        }

        // Advance timecode to next frame
        ltc_encoder_inc_timecode(enc);
    }

    // ---- Cleanup ----
    std::cout << "\nStopping.\n";
    if (wav.is_open()) {
        const uint32_t final_data_bytes = (wav_data_bytes > 0xFFFFFFFFull)
                                              ? 0xFFFFFFFFu
                                              : (uint32_t)wav_data_bytes;
        if (!write_wav_header(wav, (uint32_t)SAMPLE_RATE, (uint16_t)CHANNELS,
                              (uint16_t)BIT_DEPTH, final_data_bytes)) {
            std::cerr << "Failed to finalize WAV header\n";
        }
        wav.close();
    }
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
    ltc_encoder_free(enc);
    return 0;
}
