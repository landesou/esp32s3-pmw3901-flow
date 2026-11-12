# ESP32-S3 + PMW3901 光流传感器调试实录：ID 读到了，数据却是错的

一句话总结：用 ESP-IDF v5.5.3 在 ESP32-S3 上把逐飞科技的 PMW3901 光流模块跑通，**芯片 ID 是 0x49 / 0xB6，50Hz 运动数据正常**，并配了一套 PyQt5 上位机把 counts 实时画成 2D 轨迹。

代码仓库：**https://github.com/landesou/esp32s3-pmw3901-flow**（MIT，含固件工程 + 上位机 + 实测数据）。

配套上位机（`host/flow_host.py`）也已写完并联调通过：实机 8 秒采到 **364 包、0 坏帧、50.0Hz**，CSV 364 行，移动模块时轨迹跟得上。

值得写下来的原因：这个项目里真正花时间的不是"点亮"，而是**"代码编译通过了、ID 也读到了，但数据是错的"**这三次定位——运动标志读错位、快门寄存器分两次读导致高低字节错位、串口被别的程序抢走。这三个坑都在数据层面，看日志不仔细就会当成"传感器就这德行"。

<!-- TODO: 截图：串口 monitor 里 chip id 0x49/0xB6 + init done 的启动日志 -->

---

## 1. 为什么想用光流测速

做小车的都绕不开一个尴尬：轮式编码器测的是**轮子转多快**，不是**车相对地面跑多快**。轮子一打滑（气垫车、光滑地面、起步猛加速），编码器给出的速度就变成了"轮子在空转"，控制环越修越歪。

光流传感器解决的是另一件事：镜头朝下，看地面纹理在视野里移动了多少像素，直接输出**相对地面的位移**。它测的是"地"，不是"轮"，天然免疫打滑。

PixArt 的 PMW3901 是这类传感器里非常常见的一颗：4 线 SPI，直接输出位移增量（counts），不需要主控自己做图像处理，一颗 MCU 就能接。逐飞科技的光流模块用的就是这颗 PMW3901MB（`pmw3901.h` 头注释里也写明了这一点）。

选它的理由很朴素：**接口简单 + 资料够多**。初始化寄存器序列可以从 [Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901)（MIT）抄，寄存器语义在 LibDriver 和 ArduPilot 的实现里能交叉验证，出问题时有地方对照——这对第一次碰光流的人来说，比参数表上的几个小数点重要得多。

<!-- TODO: 截图：逐飞 PMW3901 光流模块实物（正面镜头 / 背面丝印） -->

### 它的局限，最好一开始就知道

- **输出是 counts，不是毫米**。同一个 counts 在不同对地高度对应的实际位移不一样，必须乘高度才近似成毫米；而要当速度计用，还得实测标定系数。
- **必须有纹理、有光**。纯白墙、玻璃、镜面、太暗的地面基本读不到；镜头要朝下，离地 8cm 以上。
- **视场有限**。PMW3901 视场约 42°、35×35 像素，这个数字后面算粗略换算系数要用到。

所以这个项目的定位很明确：**先把"能稳定读出数据"这件事做扎实，精度交给标定和融合去做**，而不是一上来就追求测速准。

---

## 2. 硬件接线

默认用 ESP32-S3 的 **SPI2（FSPI）默认引脚**，正点原子 DNESP32S3 这类开发板的排针上都引出来了：

| PMW3901 模块 | ESP32-S3 | 说明 |
|---|---|---|
| VCC | 3V3 | 模块若标注 5V 输入就接 5V（板载 LDO），**二选一，别同时接** |
| GND | GND | 必须共地 |
| SCK / SCL | GPIO12 | SPI 时钟 |
| MOSI / SDA | GPIO11 | 主出从入 |
| MISO / ADR | GPIO13 | 主入从出 |
| CS / CSN | GPIO10 | 片选，低有效 |
| MOT（如有） | 不接 | 运动指示引脚，本工程用轮询，不需要 |

三个容易翻车的点：

1. **逻辑电平是 3.3V**，别拿 5V 电平的 SPI 直接怼；
2. **杜邦线别太长**（<15cm），长了先把 SPI 时钟从 2MHz 降到 1MHz 排查；
3. **别和 LCD 抢 SPI2**：很多 ESP32-S3 例程里 LCD 也占 GPIO11/12/13，同时用时要么换引脚，要么把两个设备挂到同一条 SPI 总线（`spi_bus_initialize` 只调一次，再 `spi_bus_add_device`）。

<!-- TODO: 截图：接线实物（模块 + 杜邦线 + 开发板） -->

---

## 3. 环境坑：export.ps1 硬找 py3.12，而机器上装的是 py3.11

环境是 **ESP-IDF v5.5.3**，路径 `D:\ESP32-S3\Espressif\frameworks\esp-idf-v5.5.3`。按官方文档在 PowerShell 里跑 `export.ps1`，它会去找

```
python_env\idf5.5_py3.12_env
```

但实际装好的是

```
D:\ESP32-S3\Espressif\python_env\idf5.5_py3.11_env
```

目录名对不上，脚本直接失败。这属于本机 ESP-IDF 安装器留下的不一致，不是工程的问题，但会卡住第一次构建。

绕开有两条日常可用的路，按省事程度排序：

1. **VS Code + ESP-IDF 插件**（平时做例程的流程）：打开工程目录，底部状态栏选 esp32s3 和串口，Build → Flash → Monitor；
2. **开始菜单里的 "ESP-IDF 5.5 CMD"** 快捷方式，环境变量由它自己配好，直接 `idf.py build`。

想在普通命令行里完全绕开 `export.ps1` 也不是不行，但**不是加一行命令的事**：得先用 `idf_tools.py export` 把 `IDF_PATH` 和工具链的 `PATH` 导进当前 shell，再调用 `%IDF_PATH%\tools\idf.py`。步骤比较长，不适合塞进正文——日常用上面两条路就够了。

首次编译结果：**0 warning / 0 error**，bin 231,808 字节（早期版本）。修完驱动、加上 `$FLOW` 数据帧并和上位机联调之后，最终 bin 是 **245,456 字节（0x3bed0）**，1MB app 分区还剩 77%。

---

## 4. SPI 驱动：把数据手册的三条时序写成代码

PMW3901 的 SPI 不算复杂，但有三个地方和"标准 SPI 外设用法"不一样，也是最容易抄错的地方。

### 4.1 模式 3、2MHz

```c
spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = cfg->clock_hz,   /* 默认 2000000 */
    .mode           = 3,               /* CPOL=1, CPHA=1 */
    .spics_io_num   = -1,              /* 片选不用硬件 SPI 片选 */
    .queue_size     = 1,
};
```

数据手册给的上限就是 2MHz，别贪快。

### 4.2 片选必须手动控制，因为要等 tSRAD

读寄存器分两步：先发地址，**等 tSRAD（手册要求 ≥35us）**，再读数据；整个过程中 CS 必须一直保持低电平。ESP-IDF 的硬件片选在一次 transaction 结束就自动拉高了，中间插不进这个等待，所以干脆把 `spics_io_num` 设成 -1，用 GPIO 自己拉：

```c
cs_low();
esp_rom_delay_us(PMW3901_T_CS_SETUP_US);   /* 50us，CS 建立 */

err = pmw3901_spi_xfer(&addr, NULL, 1);    /* 先发地址 */
if (err == ESP_OK) {
    esp_rom_delay_us(PMW3901_T_SRAD_US);   /* 50us，手册要求 >=35us */
    err = pmw3901_spi_xfer(&dummy, value, 1);
}

esp_rom_delay_us(PMW3901_T_CS_HOLD_US);    /* 50us，CS 保持 */
cs_high();
```

工程里固化的几个时间参数（单位 us）：CS 建立 50、tSRAD 50（手册最小值 35）、CS 保持 50、连续两次写之间 200（手册要求 ≥180）。**宁可给足，别贴着最小值写**——杜邦线一长，时序余量就是唯一的容错。

### 4.3 地址最高位：读清零、写置一

这是 PMW3901 的寄存器读写规则，也是最容易抄错的一句：

- **读**：地址最高位保持 0，代码里 `reg & 0x7F`；
- **写**：地址最高位置 1，代码里 `reg | 0x80`。

搞反了的现象通常是"ID 能读到、写配置全失效"，非常隐蔽。

### 4.4 上电复位 + 73 条初始化寄存器

芯片上电后要先做 Power_Up_Reset：CS 拉高—拉低—拉高清掉移位寄存器里的残余位，然后写 `0x3A = 0x5A`，等几毫秒。

之后是那段"看起来很吓人"的初始化序列：**73 条寄存器写入**，分布在 `0x7F` 选中的多个 bank 里，中间还有一处必须停 100ms 再继续。这一段直接照抄 [Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901)（MIT License）的序列（73 条，与它逐条一致），再按 ESP-IDF 重写成 `pmw3901_write_reg()` 循环。**这 73 条不需要理解每一条在干什么**，照着写就行，少一条数据就可能不可用。

### 4.5 读数据：一次 burst 拿完 12 字节

这是整个驱动最核心的一步，也是后面第 5 节两个坑的解法：

```c
uint8_t addr = PMW3901_REG_MOTION_BURST;   /* 0x16，读地址最高位保持 0 */

cs_low();
esp_rom_delay_us(PMW3901_T_CS_SETUP_US);
err = pmw3901_spi_xfer(&addr, NULL, 1);
if (err == ESP_OK) {
    esp_rom_delay_us(PMW3901_T_SRAD_US);
    err = pmw3901_spi_xfer(dummy, burst, sizeof(burst));   /* 12 字节 */
}
esp_rom_delay_us(PMW3901_T_CS_HOLD_US);
cs_high();

motion->motion  = (burst[0] & 0x80) != 0;                    /* bit7 才是运动标志 */
motion->delta_x = (int16_t)(((uint16_t)burst[3] << 8) | burst[2]);
motion->delta_y = (int16_t)(((uint16_t)burst[5] << 8) | burst[4]);
motion->squal   = (uint8_t)(burst[6] * 4);                   /* burst 里是 SQUAL 的 1/4 */
motion->shutter = (uint16_t)(((uint16_t)(burst[10] & 0x1F) << 8) | burst[11]);
```

12 字节的布局（PMW3901MB 手册 / LibDriver 一致）：

| 下标 | 含义 |
|---|---|
| [0] | MOTION，**bit7 为运动标志** |
| [1] | Observation |
| [2] / [3] | Delta_X 低 / 高字节 |
| [4] / [5] | Delta_Y 低 / 高字节 |
| [6] | SQUAL / 4（所以读出来要乘 4） |
| [7] / [8] / [9] | RawData_Sum / Max / Min |
| [10] | Shutter 高字节（低 5 位有效） |
| [11] | Shutter 低字节 |

一次 transaction 读完的好处不是省时间，而是**这 12 个字节是同一时刻的快照**——原因见下面第二个坑。

---

## 5. 实机踩坑三连

### 坑 1：motion 帧数永远是 0，但 dx/dy 有数

第一次上电的日志大概是这样：`chip id: 0x49 / 0xB6`，`init done`，然后——

- `motion` 帧数一直不涨；
- `dx/dy` 明明会跟着手移动变化；
- `squal` 在 170~210 之间，看起来还挺正常。

看现象很容易以为是"传感器没检测到运动"，但 dx/dy 有数就说明它明明检测到了。问题出在**读运动标志的位置错了**：我最初按 MOTION 寄存器 `0x02` 的 bit0 判断运动（不少示例代码也是这么写的），而 PMW3901MB 的 burst 数据里，**运动标志在字节 0 的 bit7**。

这个错误还有个非常坑的副作用。主循环里 `motion` 标志只做两件事：累计 `motion_frames`、以及在 5 秒内一帧运动都没检测到时打印

```
no motion detected in 5 s: lens faces down? textured surface? height >= 8 cm?
```

所以读错位的时候，**dx/dy 明明是正常的，程序却在 5 秒后一本正经地提示你"镜头没朝下、地面没纹理"**。如果顺着这个提示去拆线、换桌面、调高度，方向就完全跑偏了——这也是为什么"日志出现什么提示"和"数据实际是什么"必须分开看。

改完 `motion = burst[0] & 0x80` 之后，移动模块时 `motion` 帧数从 0 一路涨到 442，dx/dy 同步响应。

<!-- TODO: 截图：motion 帧数从 0 开始增长 + 5s 误报警告的日志 -->

> 结论：判断"有没有运动"看 `0x16` burst 的 `burst[0] & 0x80`，`0x02` 的 bit0 不对。

### 坑 2：快门值在 0 / 64 / 3762 / 8191 之间乱跳

同一个首测里还有个更怪的现象：`shutter` 忽大忽小，在 0、64、3762、8191 之间跳。8191 正好是 0x1FFF（13 位上限），明显是被拉满的假值。

拆开看，这里其实叠着两个独立的问题：

1. **地址抄错了**：0x0E / 0x0F 根本不是快门寄存器（正确是 **0x0B 低字节 / 0x0C 高字节**），地址不对，读回来的值本来就没有意义；
2. **就算地址对，高低字节分两次读也可能拼错**：先读低字节、再读高字节，两次读之间寄存器可能已经被刷新，拼出来就是高字节来自"上一帧"、低字节来自"这一帧"的 Frankenstein 值。

当时是"地址 + 读法"一起改的：换成 `0x16` burst 一次读 12 字节，dx/dy/squal/shutter 全部来自同一时刻的快照。改完 `shutter` 稳定在 **2830** 附近，`squal` 稳定在 170~210（木纹桌面）。

> 诚实说明：两个改动是同时做的，**没法把"乱跳"单独归因给哪一个**。"分开读会跨帧拼错"是合理的技术推断，但当时没有做受控实验去分离这两个因素。

<!-- TODO: 截图：修好前后两段串口日志对比（shutter 乱跳 vs 稳定 2830） -->

### 坑 3：`Could not open COM6, the port is busy or doesn't exist.`

烧录时直接报错原文：

```
A fatal error occurred: Could not open COM6, the port is busy or doesn't exist.
```

先把"自己的代码/环境"排除掉：用 `tasklist` 和 PowerShell 的 `Get-CimInstance Win32_Process` 看谁占着串口，排除了几个后台进程。最后定位到的是**我自己之前写的一个 T-Display 电脑状态监视小工具**——它开机后会自动找串口连，把 CH340 的 COM6 抢走了。

处理很直接：`taskkill //PID <pid> //F` 结束那两个进程，端口立刻释放。教训是**烧录前先关掉所有自动连串口的常驻程序**，或者干脆给它固定一个别的串口。

<!-- TODO: 截图：esptool 报 Could not open COM6 的终端截图 -->

### 坑 4（小）：冷启动第一次读 ID 得到 0x00 / 0x00

模块刚上电的一小段时间里，SPI 已经通了，但芯片还没准备好，第一次读 `0x00 / 0x5F` 会得到 `0x00 / 0x00`，等一会儿再读就正常。这在初始化里加了**5 次重试、每次 20ms** 就解决了；重试 5 次还失败才返回 `ESP_ERR_NOT_FOUND`，认为模块没接好。

这个坑本身不重要，重要的是**别把它当成"接触不良"**去反复插线——ID 反码机制（`0x00` 读 0x49、`0x5F` 读 0xB6，两者互为反码）本来就是给这种场景自检用的。

---

## 6. 数据怎么验证：把"有数据"和"数据对"分开

这里有个当时真实卡住的问题：**传感器读到数了，但我怎么知道数是对的？**

"有数据"很容易验证：`chip id` 对、`motion` 帧数在涨、`dx/dy` 跟着方向变、`squal` 不为 0、`shutter` 不是 8191。这些都过了，只能说明通信和初始化是好的。

"数据对"要另外设计实验。按物理关系，counts 和地面位移的关系是：

```
位移(mm) ≈ counts × 对地高度(mm) × K
K ≈ 42° / 35 像素 ≈ 1.2° ≈ 0.021 rad ≈ 0.0209
```

在 100mm 高度下，**1 count ≈ 2.09mm**，所以直尺平移 **200mm 理论上约 96 counts**（这个数是几何近似，不是实测结果）。

当时规划的三步验证：

1. **静止零漂**：模块固定不动，对地 100mm，跑 30s，看累计 counts 漂多少；
2. **直尺平移**：沿直尺推 200mm，比对累计 counts 和理论值 96 的偏差；
3. **装车对比编码器**：小车匀速直线跑一段，光流累计位移和编码器里程对比。

> 状态说明：截至这篇初稿，**只完成了"有数据"层面的验证**（ID、运动响应、squal/shutter 稳定）。上面三步里的量化数据还没有实测结果，等做完再补。所以固件里那个 `v` 速度字段只是用几何系数算的趋势参考，**不是标定后的真实速度**。

这也直接决定了下一步要做什么：**已知距离自动标定 K**。上位机里已经把它做成了一个按钮（见 8.3），但实机推 200mm 的精度复核还没做——功能有了，数据还没验。

---

## 7. 为什么要单独发一条 `$FLOW` 数据帧

一开始数据全靠 `ESP_LOG` 打印，人看着挺爽，上位机解析起来很痛苦：日志行格式会变、有颜色控制符、有别的模块的日志混进来。所以固件侧加了一个"给人看的日志"和"给机器看的数据帧"分开发的设计：

每帧单独发一行：

```
$FLOW,<t_ms>,<dx>,<dy>,<motion>,<squal>,<shutter>*<XOR>\n
```

- `XOR` = `$` 之后、`*` 之前所有 ASCII 字符的异或，两位大写十六进制；
- 50Hz，每行约 40 字节，115200 波特率下约 2KB/s，带宽余量很大；
- 人看的 `ESP_LOG` 降到 1 秒一条，互不干扰。

为什么用 ASCII 而不是二进制帧？因为**调试阶段可读性比带宽值钱**：串口助手里直接就能看到完整一帧，校验错了肉眼也能发现；2KB/s 对 115200 波特率来说是零压力。等以后真觉得不够，再换二进制协议也不迟——先别为了省那点带宽把调试难度翻一倍。

固件里就是拼字符串 + 算校验，不到 15 行：

```c
static void flow_send_frame(int64_t t_ms, const pmw3901_motion_t *m)
{
    char body[96];
    int  len = snprintf(body, sizeof(body), "FLOW,%lld,%d,%d,%d,%u,%u",
                        (long long)t_ms, m->delta_x, m->delta_y,
                        m->motion ? 1 : 0, (unsigned)m->squal, (unsigned)m->shutter);

    uint8_t chk = 0;
    for (int i = 0; i < len; i++) {
        chk ^= (uint8_t)body[i];
    }
    printf("$%s*%02X\n", body, chk);
}
```

带校验的原因很实际：串口线一长、波特率一高，偶尔就会错一个字节。有了 XOR，上位机可以立刻把这行丢掉，而不是拿一个错位的数据去画轨迹。

---

## 8. 上位机：把 counts 变成一条能看的轨迹

固件能吐数据之后，立刻遇到新问题：**光看串口日志根本判断不了"轨迹对不对"**。dx/dy 一行一行滚过去，你没法知道它是在画圆还是在走直线。所以补了一个上位机。

需求当时定得很清楚：Python + PyQt5 + pyqtgraph，USB 串口直连（WiFi 以后再说），界面做完整版——2D 轨迹、dx/dy 实时曲线、SQUAL/Shutter、状态栏、CSV/截图导出，并且能输入对地高度和系数 K、按已知距离自动标定。

成品是单文件 `host/flow_host.py`（约 890 行），配 `requirements.txt` 和 `run_host.bat`。依赖装在独立 venv 里：PyQt5 5.15.11 / pyqtgraph 0.14.0 / pyserial 3.5 / numpy 2.4.6。`run_host.bat` 第一次运行会自动建 venv、装依赖（要联网），之后双击就直接开界面。

### 8.1 数据管线：一行数据怎么变成像素

串口线程一次读 512 字节，按 `\n` 切行，交给 `parse_flow_line()` 校验；校验通过才用 Qt 信号丢回主线程，累加 counts、乘上 `高度 × K` 变成 mm，再更新轨迹和曲线。

```python
def parse_flow_line(raw: bytes):
    line = raw.strip()
    if not line.startswith(PROTO_TAG):        # 不是 $FLOW 开头，直接丢
        return None
    if b"*" not in line:
        return None
    body, _, chk = line[1:].partition(b"*")
    try:
        if _xor8(body) != int(chk[:2], 16):   # XOR 校验，错一帧丢一帧
            return None
        fields = body.decode("ascii").split(",")
        if len(fields) != 7 or fields[0] != "FLOW":
            return None
        return (int(fields[1]), int(fields[2]), int(fields[3]),
                int(fields[4]), int(fields[5]), int(fields[6]))
    except (ValueError, UnicodeDecodeError):
        return None
```

两个刻意的设计：

- **`ESP_LOG` 日志行不算坏帧**，只有以 `$` 开头但解析失败的才算坏帧。否则每秒一条的日志会把"坏帧"计数顶满，这个数字就失去意义了；
- 解析失败**静默丢弃**，不打印。串口噪声是常态，每帧都 print 反而会拖慢主线程。

### 8.2 界面

<!-- TODO: 截图：上位机实机运行界面，原图 host/capture_real.png（上传 CSDN 后替换为图床链接） -->
> 图 1：接实机 COM6 采 8 秒——状态栏 `采样率 50.0Hz / 包 364 / 坏帧 0`，蓝线是手托模块移动留下的轨迹。来源：`host/capture_real.png`

界面是三块图 + 一行状态栏，各管一件事：

- **左：轨迹（mm）**，等比例坐标，起点在原点（绿点），红点是当前位置；
- **右上：原始位移 dx/dy（counts）**，10 秒滚动窗口，看单帧数据有没有异常；
- **右下：SQUAL / Shutter 双 Y 轴**。快门是几千的量级、squal 只有两百，共用一根轴会把 squal 压成一条直线，所以拆成左右两个纵轴；
- **状态栏**：采样率、包数、坏帧数、坐标、速度、累计距离、当前 squal/shutter。

参数栏提供对地高度、系数 K、翻转 X/Y、清零、标定 K、暂停、跟随视图、保存 CSV、截图。高度、K、翻转、波特率这些设置会持久化，下次打开还在。

实机跑 8 秒的截图里，状态栏是 `采样率 50.0 Hz / 包 364 / 坏帧 0`，左边那条蓝线就是用手托着模块在桌面上来回移动的轨迹——形状当然是乱的，但**每一段的方向都和手的移动方向对得上**，这就是"数据通到界面"的证据。

### 8.3 按已知距离自动标定 K

这是前面第 6 节留下的问题的正式解法。点"标定 K"打开对话框：点【开始记录】记下当前 counts → 沿直尺推一段已知距离 → 填距离 → 点【计算并应用】。公式就是

```
K = 距离 / (counts × 高度)
```

算出来的 K 直接写回界面并持久化。比手动算 42°/35 靠谱得多，也把"实机推 200mm 验证"变成了一个按钮的事。

### 8.4 怎么验证上位机是好的

不用盯着界面猜，分三层，越往下越接近真实：

1. **协议自检（不需要硬件）**：`PROTOCOL_SELFTEST_OK`——正常帧必须解析成功，校验错和垃圾数据必须被拒绝；
2. **离线自检（不需要硬件）**：`--selftest out.png` 喂 10.5 秒合成数据（50Hz × 10.5s = 525 个点），输出 `SELFTEST_OK samples=525 rate=50.0Hz csv_rows=525`，界面状态栏同步显示 50.0 Hz；
3. **实机联调**：

| 场景 | 方法 | 结果 |
|---|---|---|
| 串口直读 4 秒 | 直接统计串口输出 | `OK_FRAMES=186 BAD=0 ESP_LOG_LINES=67`（≈47Hz，日志行被正确忽略） |
| 上位机接实机 8 秒 | `--port COM6 --capture 8 --shot capture_real.png` | `packets=364 bad=0 rate=50.0Hz`，CSV 364 行，移动模块时轨迹跟随 |
| 演示模式 5 秒 | `--demo --capture 5` | `packets=161 bad=0 rate=50.0Hz` |

`--capture N --shot out.png` 这条命令是专门为联调加的：采 N 秒 → 自动截图 → 存 CSV → 打印包数/坏帧/采样率，然后退出。以后改代码，一条命令就能做回归，不用再手动点界面。

### 8.5 上位机踩的 4 个小坑

1. **`QSettings` 在 `QtCore` 里，不在 `QtWidgets` 里**。`from PyQt5 import QtWidgets` 之后顺手写 `QtWidgets.QSettings`，会直接报属性不存在。这个错误很低级，但 PyQt 的命名空间分散在 QtCore/QtGui/QtWidgets 三个模块，第一次写很容易踩。
2. **忘了 `import csv`**：界面能开、数据能画，直到点"保存 CSV"那一刻才抛 `NameError`。教训是**导出类功能必须单独点一遍**，只测"能连上能画图"会漏掉。
3. **采样率用本机时钟算，离屏自检显示 0Hz**。第一版用本机 `time.monotonic()` 统计帧率，正常使用时没问题；但 `--selftest` 是瞬间灌 525 个样本，本机时钟几乎没走，算出来就是 0（或者一个巨大的假值）。改成用设备时间戳 `t_ms` 之后，自检和实机都能稳定显示 50.0 Hz——**这也说明时间戳应该由数据源给，而不是由接收端猜**。
4. **演示数据的整数 counts 把圆周画成了多边形**。合成圆周每帧位移只有 1~2 counts，`int(round())` 之后误差会累积成锯齿。解决办法是"误差扩散"：把这一帧舍入丢掉的小数加到下一帧再取整。截图里圆边缘还有一点台阶，那是 1 count ≈ 2.09mm 的物理分辨率决定的，不是 bug。

<!-- TODO: 截图：`--demo` 合成圆周（原图 host/demo_capture.png，上传 CSDN 后替换为图床链接） -->
> 图 2：演示模式 5 秒，`packets=161 bad=0`。圆边缘的细微台阶就是整数 counts 量化留下的痕迹。来源：`host/demo_capture.png`

---

## 9. 局限与下一步

这个工程现在**固件 + 数据通道 + 上位机都已验证**，但离"能测速"还有距离，诚实列一下：

1. **counts → mm 用的是几何近似**，随镜头批次、地面纹理、安装高度变化，必须标定；
2. **没有滤波**：光流原始数据抖动明显，做速度环之前先加一阶低通；
3. **50Hz 轮询**：`SAMPLE_PERIOD_MS` 改成 10ms 就是 100Hz，burst 读一轮只要约 0.3ms，完全带得动；更省 CPU 的做法是接 MOT 引脚中断；
4. **高度不是常数**：小车颠簸时对地高度在变，想做成靠谱的速度计，得配 VL53L1X 之类测高传感器做融合；
5. **量化验证还没做完**：零漂 / 直尺 200mm / 装车对比编码器仍然只有方案没有数据（上位机的自动标定 K 功能已经可用，但实机推 200mm 的复核还没跑），见第 6 节。

---

## 10. 想复现的话

1. 按第 2 节的表接线，注意 3.3V 电平和别和 LCD 抢 SPI2；
2. 用 ESP-IDF v5.5.3 打开工程，`idf.py set-target esp32s3` 然后 `idf.py build`（本工程实测 0 warning / 0 error）；
3. `idf.py -p COMx flash monitor`，串口 115200；
4. 启动日志里应该能看到 `chip id: 0x49 / 0xB6` 和 `init done`；第一次出现 `0x00 / 0x00` 不用慌，驱动会自己重试；
5. 手在模块下方移动，看 `motion` 帧数增长、`dx/dy` 跟着方向变——到这一步，固件端的通道就算通了；
6. 上位机：双击 `host\run_host.bat`（首次会自动建 venv 装依赖），或者 `cd host` 后 `.venv\Scripts\python.exe flow_host.py --port COM6`；手边没有板子的话先 `--demo` 看界面。

## 11. 参考资料

- PixArt **PMW3901MB** 数据手册（寄存器表、SPI 时序、Power_Up_Reset）
- [Bitcraze_PMW3901](https://github.com/bitcraze/Bitcraze_PMW3901)（MIT License）：本工程初始化寄存器序列的来源，已按 ESP-IDF 重写
- [逐飞科技文档中心](https://www.seekfree.com.cn)：模块引脚以实物丝印为准
- [基于逐飞 AI8051U 开源库移植 PMW3901 的帖子](https://www.stcaimcu.com/archiver/tid-17873.html?fid-87.html)：寄存器地址读写规则的说明
