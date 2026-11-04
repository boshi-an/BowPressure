# Bow V2 / V3 固件

只保留四个固件版本：v2、v2_no_ltc、v3、v3_no_ltc。两版共用采集和协议代码，硬件差异由 `config/v2.h`、`config/v3.h` 选择。此目录不包含 V4 差分通信硬件配置。

## 目录

```text
config/          v2.h / v3.h 硬件版本入口
src/             共用 MCU、IMU、ADC、LTC、串口代码
host/            read.py 原始数据读取；generate_ltc.py 测试音频生成
build/           四个版本各自的 hex/bin/elf/map
tests/          主机协议、LTC、固件逻辑测试
vendor/          STM32/CMSIS 头文件与许可证
out/             原有录制数据，清理和编译均不删除
test_signals/   LTC 测试音频
```

## 版本区别

| 固件 | 硬件 | IMU调度 | LTC |
|---|---|---|---|
| `v2` | v2.epro2 | 轮询 | 开启 |
| `v2_no_ltc` | v2.epro2 | 轮询 | 关闭PA0边沿中断 |
| `v3` | v3.epro2 | PA6 FIFO中断 + 兜底轮询 | 开启 |
| `v3_no_ltc` | v3.epro2 | PA6 FIFO中断 + 兜底轮询 | 关闭PA0边沿中断 |

四版均为IMU1600Hz、20位HIRES（固定±32g/±4000°/s比例）、AAF和400Hz UI LPF；
ADC Mode3/OSR256/PGA32/AVDD参考、双桥各1000SPS；SPI默认3MHz；UART1500000/8N1。
电脑端不额外平滑。V3启用PA4状态LED，V2不启用。

`no_ltc`版保留相同传感器采集和全量传输，只屏蔽LTC EXTI边沿触发及对应NVIC中断；
PA0仍为无上下拉输入。LTC心跳保持未锁定、计数为0，显示 `LTC=UNLOCKED` 属于正常现象。
启动日志包含 `# LTC disabled`。外部LTC悬空曾导致中断风暴和IMU FIFO满，
不用LTC时请选择no_ltc版；启用LTC前应确保输入稳定。

V3 INT1：U2.4→U3.13/PA6，两帧水位（40字节），推挽、高有效、锁存；2048字节FIFO约容102帧。中断只置标志，主循环每次最多读4帧，逐帧让行ADC；ADC到期前200us暂缓IMU。水位等待使最早一帧增加约0.625ms传输等待，不改变1600Hz采样率。`mcu_read_us`是读取时间，IMU采样间隔请参考FIFO传感器时间戳。

V3启动后未收到INT1中断会在状态报告中置 `faults & 0x80`（128），仍通过兜底轮询回传，故障位锁存至复位。这不是FIFO溢出，也不能证明后续每次中断都正常。ADC/IMU故障、TX错误或丢包也触发快闪；未接LTC不触发故障灯。

## 编译与测试

需要 `arm-none-eabi-gcc`；Python测试在已有 `stm32` conda环境中运行。

```sh
conda activate stm32
make             # 同时构建四个版本
make v2          # 仅构建 V2
make v3          # 仅构建 V3
make v2_no_ltc   # V2，关闭LTC
make v3_no_ltc   # V3，关闭LTC
make test        # 协议、LTC、调度/ADC/DMA/LED逻辑测试
make clean       # 只删除 build，不删除录制数据
```

可用 `make v3 SPI_BR=4` 将SPI改为1.5MHz；默认 `SPI_BR=3` 为3MHz。每次构建明确输出版本名，不再提供 accel/adc/imu/sensors/fast 旧入口。

## 接线与烧录

V2/V3的J1相同：

| 板端 | 电脑端 |
|---|---|
| J1.1 LTC | 音频LTC信号；源地与板地共地 |
| J1.2 GND | USB-TTL GND |
| J1.3 DATA | USB-TTL RX，3.3V逻辑；适配器TX不接 |
| J1.4 5V | USB 5V电源输出，不能并联两个电源输出 |

J3：1=GND、2=3V3目标参考、3=NRST、4=SWCLK、5=SWDIO。J3.2只接ST-LINK明确的VTref/VAPP输入；若调试器3.3V脚是电源输出，不连接该脚。通过J1供电。

用STM32CubeProgrammer通过SWD连接STM32C011，选择匹配硬件和LTC选项的 `build/<版本>.hex`，下载并Verify后复位运行。BIN的Flash起始地址为 `0x08000000`。程序不会自动烧录。

## 电脑端读取

四个版本共用 `host/read.py`，波特率均为1500000。

```sh
conda activate stm32
python host/read.py --list
python host/read.py --port /dev/cu.usbserial-BG03EL9E --baud 1500000 --diagnostics
```

保存所有数据到一个共享CSV（文件必须尚未存在，父目录自动创建）：

```sh
python host/read.py --port /dev/cu.usbserial-BG03EL9E --diagnostics --duration 60 \
  --csv out/record01.csv
```

四个固件版本都使用同一保存格式，`--csv`不再仅代表IMU。
旧的 `--adc-csv`、`--status-csv`、`--ltc-csv` 参数已移除。
每个有效数据包单独一行，按接收顺序保存，不插值、不补旧值、不额外平滑。
`record_type`区分：

| 类型 | 内容 |
|---|---|
| IMU | 六轴原始值、g/°/s换算值、传感器时间戳 |
| ADC | channel=H/F、raw24、diff_mV、增益、AVDD换算假设、ADC状态、转换起始与观测延迟 |
| LTC | 时间码、帧率、锁定、边沿及解码错误/溢出累计 |
| STATUS | TX丢帧、ADC错过时隙、最大延迟、故障、存活标志 |
| IMU_STATUS | FIFO当前/峰值、INT1状态、电平、中断/查询数、最后查询时刻 |
| LOG | 捕获到的启动文本，无MCU时间戳；写入位置不用于精确排序复位事件 |

数据行共享host_unix_s、epoch、seq、mcu_read_us、mcu_read_us_unwrapped，
并记录主机CRC/序号/IMU间隔/无效载荷累计计数。不适用列留空；seq属于各包类型，
ADC的H/F共用序号。epoch区分检测到的MCU复位；展开时间在epoch内使用。
电脑时间是解析时刻，不能替代采样时刻。CRC失败的包不作为有效读数保存。
CSV每秒刷新，Ctrl+C正常关闭。无需tee；已有录制文件不会被覆盖。
no_ltc版本仍会保存LTC未锁定心跳。未指定--csv时仅显示，不保存。

默认显示 |a|、|v_ang|、H(mV)、F(mV)；`--diagnostics`额外显示诊断。端口名按电脑实际设备选择。音频测试可播放 `test_signals/ltc_25fps_60s.wav`；不要同时用其他串口软件占用端口。

## 诊断与验证

`--diagnostics`显示FIFO水位、故障、采样率、CRC、TX丢帧及ADC调度延迟。
故障4表示检测到FIFO满或计数>=102，之后停止IMU读取并保留现场，需要复位恢复。
捕获启动日志：先启动read.py，再短暂将J3.3(NRST)接J3.1(GND)并松开。
未接应变桥时ADC可能饱和或漂移；仅测半桥时查看H，全桥F可忽略。

本次四版编译通过，16项测试通过（包括5组原生C测试）；新增v2_no_ltc尚待实板验证。
临时v3_diag、v3_ltc_test及其主机专用脚本/测试已移除，历史out数据保留。
保留generate_ltc.py作为有效LTC音频生成工具、test_signals音频和通用自动测试。

## 绘制统一CSV

stm32环境需要matplotlib（已列入environment.yml；缺少时运行 `python -m pip install matplotlib`）。
在firmware目录运行：

```sh
python host/plot.py out/record01.csv --show
```

默认保存同目录同名PNG；`--show`同时打开交互窗口，不加则只保存图片。
可用 `--output out/record01.pdf` 指定PNG/PDF/SVG输出。
四行共享秒时间轴：ACC三轴(g)、GYRO三轴(deg/s)、半桥H(mV)、全桥F(mV)。
X/Y/Z分别为蓝/橙/绿；不平滑、不重采样。每行纵轴取该行所有曲线的全局min/max，
上下各扩展极差的10%，即总高度为极差的120%。恒定值使用小范围留白避免零高度。
横轴基于展开的MCU读取时间，减去最早传感器数据时间；不代表精确IMU采样时刻。
若录制包含复位，脚本提示运行段编号，用 `--epoch 0` 等选择，避免错误拼接。
缺失通道显示No data；无效或非有限的传感器行跳过并报告数量。
