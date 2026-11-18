# UART 机器人升降平台使用手册

> 文件名沿用历史名称，当前版本已经取消 CAN 控制。
>
> 适用固件：`lifting_platform_servo` v31
>
> UART 协议：v1，115200 bit/s，8N1，二进制帧，CRC-16/CCITT-FALSE
>
> 标定格式：v6
>
> 工作范围：以保存的中心为 0 mm，`-180.000 mm`～`+180.000 mm`

本文是升降平台模块的完整接线、标定、UART 控制、位置反馈和故障处理说明。控制板只保留中心光耦 PB14 和抱闸继电器 PA6。位置、上下边界、上电恢复和端部减速均依据电机电池保持的多圈绝对编码器，不依赖 2、4、5 号光耦。

面向RDK X5/USB主控的新交付文档：[UART机器人升降平台外设接口手册](UART机器人升降平台外设接口_RDKX5.md)。该文档包含全部帧格式、命令、反馈和完整Python客户端，可单独交付。本文继续保留维修、SWD和标定恢复说明。

## 1. 本版变化

- v29新增锁闸停稳RESET、INFO25就绪查询及INFO26～30启动/通信诊断；电机晚于MCU上电时持续等待并恢复绝对参考。
- CAN 外设和 CAN FreeRTOS 任务已从升降平台工程中关闭。
- PB8、PB9 不再用于本模块通信；机器人主控无需连接 CANH/CANL。
- 主控通过 USART1 的 PA9/PA10 直接控制。
- 旧的 ASCII 文本串口命令已经取消，PA9/PA10 只收发本文规定的二进制 UART 帧。
- 固件每 100 ms 主动发送一组四页遥测，约 10 组/秒。
- 对外目标采用 µm 整数，合法范围固定为 `-180000～+180000 µm`。
- 控制会话需要心跳；500 ms 未收到有效心跳，固件撤销 UART 控制权并停止 UART 发起的运动。

## 2. 接线

### 2.1 引脚表

| 功能 | STM32 引脚 | 方向/有效电平 | 说明 |
|---|---:|---|---|
| 主控 UART TX | PA9 / USART1_TX | MCU 输出 | 接主控 RX |
| 主控 UART RX | PA10 / USART1_RX | MCU 输入 | 接主控 TX |
| UART 公共地 | GND | — | 必须与主控共地 |
| 中心光耦 3 | PB14 | 高电平有效，输入下拉 | 只用于保存中心 `SET_CENTER` |
| 抱闸继电器 | PA6 | 高电平输出 | 高=继电器吸合并松闸，低=锁闸 |
| 手动下降 K1 | PB4 | 低电平有效 | 按住向下 |
| 手动上升 K2 | PB5 | 低电平有效 | 按住向上 |
| 电机 RS485 TX | PA2 / USART2_TX | MCU 输出 | 通过板载 SP3485 到驱动器 |
| 电机 RS485 RX | PA3 / USART2_RX | MCU 输入 | 通过板载 SP3485 到驱动器 |
| RS485 DE | PA4 | MCU 输出 | RS485 收发方向 |

PB13已改为LED2输出，不能连接光耦或其他输出；PB15、PA5不作位置传感器输入，可以不接。PB8、PB9 和板载 CAN 收发器不再参与软件控制。

### 板载指示灯（v31）

| 指示灯 | 引脚 | 闪烁含义 | 周期 |
|---|---|---|---|
| LED1 | PB12，低电平点亮 | 电机RS485通信正常；尚未成功通信、检测到通信错误或成功结果已过期时熄灭 | 1 Hz：亮500 ms、灭500 ms |
| LED2 | PB13，低电平点亮 | MCU的FreeRTOS指示灯任务持续运行，与电机通信状态无关 | 1 Hz：亮500 ms、灭500 ms |

LED1由最近一次已记录的电机操作结果更新：成功则允许闪烁，失败则在下一次灯光刷新时熄灭；超过1500 ms没有新的成功结果也熄灭，防止旧成功状态一直保留。刷新周期20 ms，通信断线首先需要由超时检测出来。通信恢复后重新加入闪烁，相位与LED2相同。灯光只用于观察，主控仍须检查UART的READY、位置有效位和故障码。

LED2由独立任务驱动，电机断电、等待通信或无位置参考时仍可闪烁。它表明MCU的调度和该任务仍在运行，不代表所有控制功能均无故障。上电GPIO初始化时两灯先灭，调度启动后开始上述指示。

两颗灯均不再显示抱闸。抱闸仍由PA6控制，通过UART遥测flags.bit2查看松闸输出状态；该位也不是机械抱闸的独立反馈。PB13为LED输出，不能接光耦或其他输出信号。PB14仍连接中心光耦，LED3不参与程序指示；它可能随光耦电平变化，无需改线。

v31仅更新指示任务与版本号，UART协议和中心参数格式保持兼容。下文v29真机验收记录属于此前版本，不作为v31灯光的真机验收记录。

2026-09-29已实际烧录v31并通过下载校验，中心标定区前后完全一致。串口确认版本31、0 mm、锁闸、READY=1且电机通信错误计数0；SWD连续61组GPIO读回确认LED1/LED2同相约1 Hz闪烁、PB14保持输入。本次未做电机断电/拔线测试，也未做灯珠光学测量。详见[烧录与LED测试记录](test_records/2026-09-29-v31/README.md)。

### 2.2 UART 电气连接

UART 是 3.3 V TTL 电平，不是 RS232 电平：

```text
机器人主控 TX  ─────>  PA10 / STM32 RX
机器人主控 RX  <─────  PA9  / STM32 TX
机器人主控 GND ─────  控制板 GND
```

只允许一个设备驱动 PA10。若板载 CH340 的 TX 与 PA10 相连，机器人主控直接连接时必须断开或禁用 CH340，避免两个输出端互相驱动。调试电脑使用板载 CH340 时，不要同时把机器人主控 TX 接到 PA10。

UART 参数固定为：

```text
115200 bit/s，8 数据位，1 停止位，无校验，无硬件流控
```

### 2.3 中心光耦

- 光耦输出接 PB14，光耦地与控制板 GND 共地。
- 触发时必须输出高电平，离开中心后输出低电平。
- 推荐开集电极输出上拉到 3.3 V，或直接使用 3.3 V 逻辑输出。
- 高电平确认时间 20 ms，低电平确认时间 50 ms。
- PB14 只在保存中心时校验。标定完成后的移动和限位不读取它作为停止条件。

### 2.4 抱闸继电器

PA6 上电先输出低电平，使抱闸保持锁定。固件配置驱动器、发送零速度、读取位置并确认静止后，只有确需运动时才把 PA6 拉高松闸。到达目标并确认停止后，PA6 回到低电平锁闸。

### 2.5 SWD 下载与调试

- PA13 保留给 SWDIO，PA14 保留给 SWCLK，不接其他外设。中心光耦使用的是 **PB14**。
- ST-LINK 连接 SWDIO、SWCLK 和 GND；接入 NRST 可供复位下连接使用。
- 升降项目在 `bsp_pin.h` 中启用 `BSP_DEBUG_KEEP_SWD=1`。`BSP_Init()` 在外设初始化和任务启动之前打开 AFIO 时钟，配置 `GPIO_Remap_SWJ_JTAGDisable`，关闭 JTAG、保留 SWD。按键初始化保持相同配置。
- 正常运行保持 BOOT0=0。若用户程序运行时无法连接，可在电机断电条件下设置 BOOT0=1、BOOT1=0 后复位，跳过应用进行恢复；烧录完成后恢复 BOOT0=0。
- 下载前备份 `0x0800F800～0x0800FFFF` 标定区，不执行全片擦除。以下载校验和程序运行后的再次连接验证结果，不能仅靠 LED 或读取 SWJ 配置位判断。
- 多板连接时，必须确认 ST-LINK 和调试串口对应同一 MCU；暂停目标后串口仍持续产生新遥测时，应先检查串口数据来源。

### 2.6 无法识别、下载失败或烧录后失联的处理

先区分错误发生在哪一层，不能看到“无法连接”就判断 MCU 损坏或中心参数丢失：

| 工具表现 | 含义与下一步 |
|---|---|
| `No debug probe detected` | 电脑未识别到 ST-LINK 探针；先检查探针 USB 连接、驱动和占用。此时还没有测试到 MCU，调整 BOOT 不能解决探针枚举问题。 |
| 能显示 ST-LINK，但 `No STM32 target found` / `Unable to get core ID` | 探针可见，但目标连接失败；核对供电、共地、SWDIO/PA13、SWCLK/PA14、NRST及目标板身份，按下面的恢复流程操作。 |
| 已读到芯片 ID，但 `failed to erase memory` | 目标识别与 Flash 写入是两个阶段；先保留备份，改用100kHz、复位下连接重试，不通过全片擦除或更改读保护来试错。 |
| UART有反馈，但电机返回 `0x06` | 这是电机通信链路的错误，不等于 SWD 失效。电机断电时出现该结果是预期现象；电机上电后需另查RS485。 |

**恢复与重新烧录步骤：**

1. 关闭电机主电源，保留 MCU 和 ST-LINK 供电。旧标定可能开启自动回中；调试连接恢复前不启动机构。
2. 只保留当前目标板，确认探针和串口接到同一颗 MCU。记录 MCU 的96位 UID、下载文件及 SHA256。Windows 的 COM 号会变化，不能仅凭“COM5”认定目标身份。
3. 在 STM32CubeProgrammer 中选择 **ST-LINK、SWD、100kHz、Under Reset、Hardware reset**，并接好 NRST。本次实际成功烧录使用的是这组设置。
4. 如果仍无法连接，断开供电后用板上跳帽设置 **BOOT0=1、BOOT1=0**，重新给 MCU 供电并复位，跳过用户程序进入系统启动程序，再尝试连接。BOOT选择本身不擦除Flash；LED变化只能作为现象，不能作为启动成功的判据。
5. 连接成功后先备份 `0x0800F800` 起的 `2048` 字节；能读取时建议同时备份 `0x08000000` 起的 `65536` 字节。不要先全片擦除。
6. 下载当前工程的 `.bin` 到 **`0x08000000`**，启用下载校验，仅擦除应用占用的页。保持内核暂停，读取标定区并与烧录前备份比较；不修改选项字节或读保护设置。解除读保护可能擦除Flash，不应当成常规恢复操作。
7. 校验通过后恢复 **BOOT0=0**、BOOT1保持0。在电机仍断电时复位运行，等待至少2秒，再以 **Hot Plug、100kHz** 读取芯片和程序运行状态；重复几轮，确认应用运行后仍能连接。
8. 再做同板 UART 握手及状态测试。核实保存中心确实对应当前电机和机械基准后，才进入电机上电、绝对位置确认和回中测试。

命令行示例（先把 `$programmer` 改为电脑实际安装路径，文件名按现场备份命名；每条命令成功后再执行下一条）：

```powershell
$programmer = 'C:\实际安装目录\STM32_Programmer_CLI.exe'

# 读取标定备份；使用已经能成功连接的模式。
& $programmer -c port=SWD mode=HOTPLUG freq=100 -u 0x0800F800 2048 .\calibration_before.bin

# 复位下下载并校验，结束时暂停，不自动启动机构。
& $programmer -c port=SWD mode=UR reset=HWrst freq=100 -w .\lifting_platform_servo.bin 0x08000000 -v -halt

# 校验标定区未被下载过程改变。
& $programmer -c port=SWD mode=HOTPLUG freq=100 -u 0x0800F800 2048 .\calibration_after.bin
Get-FileHash .\calibration_before.bin, .\calibration_after.bin -Algorithm SHA256
```

若 SWD 仍无法使用，可考虑系统启动程序的 **USART1串口下载**：BOOT0=1、BOOT1=0，使用3.3V TTL电平，主机TX接PA10、RX接PA9并共地，在CubeProgrammer选择UART、115200、8E1。它使用芯片自带下载协议，**不是本手册的A5 5A应用控制帧，也不是电机的RS485接口**。同样需要先读取并保存标定区、下载校验后恢复BOOT0=0。串口下载是替代恢复通道，不能代替检查固件的SWD配置；本次已通过ST-LINK完成下载，没有进行串口下载验收。[ST系统启动程序说明AN2606](https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)

**本次验证范围：**2026-09-28烧录的早期SWD配置固件下载校验通过；三轮软件复位后，等待2秒均能重新Hot Plug连接。原按键初始化也使用“关闭JTAG、保留SWD”，本次将相同配置提前到板级启动，是对启动流程的加固，不能据此断言此前所有失联均由“未打开SWD”造成。三轮软件复位不等于整机断电重启，也不代表电机通信、实际回中或运动已经验收。

## 3. 位置和速度

中心标定时把电机当前绝对计数保存为 0 mm。当前换算参数为：

```text
1 转 = 10000 电机命令单位
机械初始估算 1 转 = 1 mm
1 mm = 10000 电机命令单位
1 µm = 10 电机命令单位
```

目标范围严格固定为 ±180 mm，不会自动截断。若读取到的当前位置在 ±180 mm 之外但仍在 `-200～+200 mm` 恢复窗口内，只允许以 120 rpm 向最近的 ±180 mm 边界返回，禁止继续向外。

正常范围内采用驱动器原生PP梯形加减速：**一次下发最终目标，途中不停车、不重新下发下一段**。最高指令3000 rpm，可用 SPEED 下调。短行程自然形成三角速度曲线，不保证达到3000 rpm。

| 参数 | 驱动器寄存器 | 数值 | 按暂定1 mm/圈换算 |
|---|---|---:|---|
| 最大速度 | P10-23 / `0x1017` | 500000单位/s | 3000 rpm，50 mm/s |
| 本次PP速度 | P10-25 / `0x1019` | 由SPEED设置，最高500000单位/s | 最高3000 rpm |
| 加速度 | P10-27 / `0x101B` | 500000单位/s² | 50 mm/s² |
| 正常到位减速度 | P10-29 / `0x101D` | 200000单位/s² | 20 mm/s² |
| 中断/急停减速度 | P10-31 / `0x101F` | 500000单位/s² | 50 mm/s² |

满速50 mm/s时，正常制动距离为 `v²/(2a)=62.5 mm`。例如0→+180 mm，在有足够行程达到巡航时约从+117.5 mm开始连续减速到目标；不再在+120、+160 mm停稳重启。减速距离由当次速度和剩余行程决定，降低SPEED后会缩短。HOME、UART位置命令和按键范围内运动共用这一方式。

运动中收到STOP、失去主控心跳、按键释放或触发保护时使用P10-31的快速停止斜坡；抱闸仍待反馈确认停稳后锁定。驱动器通信失效时按故障流程处理。软件STOP不是瞬时机械制动，不能代替实体急停。只有在±180 mm外、±200 mm恢复窗口内向内返回时，仍保留120 rpm恢复段，回到工作区边界停稳后才开始正常任务。

## 4. 上电与自动回中

上电流程：

1. PA6 保持低，抱闸锁定。
2. 初始化驱动器通信并发送零速度。
3. 读取多圈绝对计数、单圈计数、编码器分辨率和电机型号。
4. 核对保存的中心配对和方向，确认速度为零且位置稳定。
5. 恢复相对高度。
6. 若已启用 `BOOT_HOME`，按同一原生PP加减速方式移动到 0 mm；实际开始运动前才松闸。
7. 到达中心、确认停止并锁闸。

绝对位置无效、驱动器通信失败、当前位置超出恢复窗口或运动监控异常时，自动运动不会开始。固件不会寻找端部光耦重新建立坐标。

### v29 上电顺序与复位

MCU/USB先上电时保持锁闸、遥测继续，等待电机上线；驱动配置成功后仍会重试绝对参考恢复，避免一次临时读数失败就永久失去自动回中。电机先上电时走同一启动流程。absolute_stage=2且BOOT_HOME=1时，恢复参考后只执行一次启动回中；INFO25=1与新鲜、有效、停稳锁闸遥测共同表示可接下一目标。

主控程序后连接不触发重设中心或重复回中。运行中驱动断电/断连恢复后重新建立参考，但不自动续跑旧任务，也不重放开机回中；需要主控明确发送HOME或新目标。STOP或按键干预会取消本次启动回中。

HOME为回到原中心；RESET为`0x1A,value=0x52535421`，只在锁闸停稳、无待执行任务时接受。成功ACK后至少250 ms执行MCU复位，Flash中心不变，旧UART会话失效；BOOT_HOME开启时随后自动回中。运动中返回0x83，STOP可取消尚未执行的复位。以uptime重置确认复位实际发生，重新建立客户端与会话。详细使用流程见新的RDK X5接口手册。

## 5. 首次标定

### 5.1 何时需要重新设置中心

新 MCU 未恢复可信备份、整片擦除后无可信备份，或更换电机/驱动器、机械中心变化时，按以下顺序操作。普通应用升级且标定完整时不需要重复设置中心。

1. 保持编码器电池连接。
2. 用驱动器调试方式或已有有效位置控制把机构置于中心光耦正中，确认 PB14 持续为高。
3. 主控建立 UART 会话并发送 `SET_CENTER`。成功后中心为 0 mm，`absolute_stage=1`。
4. 移动离开中心至少 1 mm，推荐到 `+20 mm`，停稳后发送 `ABS_CHECK`。
5. `ABS_CHECK` 核对多圈绝对增量和电机位置增量；成功后 `absolute_stage=2`，并保存向上方向。
6. 发送 `HOME` 或目标 0 mm 回到中心。
7. 发送 `BOOT_HOME`，value=1，启用每次上电自动回中。
8. 读取 INFO 页 2～6，确认范围、stage、标志和速度。

`SET_CENTER` 只有在 PB14 原始值和消抖值均为高、驱动器静止、绝对计数有效时才成功。标定成功后无需每次开机重新设置。

旧 v5 标定会在加载时自动迁移为RAM中的v6配置：保留中心、绝对零点、方向配对、巡航速度和自动回中设置；旧 2/4/5 光耦边界及其他保存点被丢弃，范围改为固定 ±180 mm。加载本身不改写Flash，下次成功保存配置时才写入v6记录。

### 5.2 中心参数的存储、备份及恢复

标定保存在：

```text
0x0800F800～0x0800FBFF  SLOT_A
0x0800FC00～0x0800FFFF  SLOT_B
总备份长度 0x800 字节
```

普通应用程序烧录不应擦除这两个页面。更换 MCU 时可以恢复同一电机、同一驱动器、同一机械中心的 0x800 字节备份；如果电机或机械基准变化，必须重新标定。

电机编码器电池保持的是绝对计数，STM32 Flash保存的是“哪一个绝对计数对应中心0mm”及方向配对。两者缺一不可：新MCU的空白Flash不会自动获得旧中心，换一块随机旧板也不能默认使用其保存中心。仅当备份属于当前机构、编码器计数未被清零或丢失时，恢复备份才有意义。

恢复可信备份时保持电机断电，先备份目标板原标定，再将完整2048字节文件写入 `0x0800F800` 并校验；不要只填写一个中心数值或手动更改CRC。恢复后重新读取INFO页4、5、6、9，确认方向阶段、自动回中标志、速度和保存点掩码；电机通信正常时再用ABS_READ和新鲜遥测确认绝对位置有效。INFO页12返回0只表示中心点在坐标系中定义为0，不能单独证明实际机构已经在中心。

### 5.3 2026-09-28此次烧录的中心保留记录

**结论：本次烧录没有丢失或重设原中心参数。**烧录前完整Flash备份的末2048字节，与烧录后读回标定区逐字节相同；两个槽记录的CRC均正确，采用序号较新的槽A。

| 项目 | 本次读回值 |
|---|---|
| 目标MCU UID（按三个32位字记录） | `066EFF49 48825550 67134555` |
| 最新标定记录 | SLOT_A，`0x0800F800`，Flash格式v5，序号17 |
| 中心保存掩码 | `0x04`，3号中心已保存 |
| 中心绝对编码器计数 | `8437940120`（`0x1F6F0BF98`）；这是内部参考计数，不是UART目标高度 |
| 编码器每圈分辨率 / 电机代码 | `131072` / `10323` |
| 绝对方向配对 | `absolute_stage=2`，`absolute_up_sign=+1` |
| 巡航速度 | `3000 rpm` |
| 旧Flash配置标志 | `0x0F`，包含自动回中；加载为v6后保留bit3，即`0x08` |
| 中心高度 / 工作范围 | `0 mm` / `−180～+180 mm` |
| 标定区SHA256 | `727d88bcccbdd8ee1c90ae4138741db10756fde88c695d5d9a4ef2f411aa4911` |
| 新固件SHA256（20556字节） | `52609d487d34004897276d304803cca5782e5d368c3519b83965c90e0de5cf32` |

已将此次2048字节标定读回另存为[随文备份](backups/calibration_066EFF49_48825550_67134555_20260928.bin)。该文件是本次目标板的标定备份，不是可以给任意平台使用的默认值；交付时应与本手册一起妥善保存，文档中的数值表不能替代完整二进制备份。

这项结论依据本次**已成功烧录后的读回**。整理文档时电脑返回`No debug probe detected`，未能追加一次板上读取，不应将此情况写成MCU再次失联。当前记录证明标定数据保留，尚未证明该旧板中心与当前实际机构对应、整机断电后回中或电机运动通过；同一机构且绝对计数连续时无需因本次烧录重新发送SET_CENTER。

## 6. UART 帧格式

### 6.1 通用帧

```text
Byte 0      0xA5
Byte 1      0x5A
Byte 2      LEN，TYPE+DATA 的字节数
Byte 3      TYPE
Byte 4...   DATA
最后2字节   CRC16，低字节在前
```

- CRC 算法：CRC-16/CCITT-FALSE。
- 多项式：`0x1021`。
- 初值：`0xFFFF`。
- 不反射，结果不异或。
- CRC 覆盖范围：从 LEN 开始，到 DATA 最后一个字节结束，不包含 `A5 5A` 和 CRC 本身。
- 多字节整数均为小端。
- 最大 DATA 长度为 16 字节。

TYPE 定义：

| TYPE | 方向 | DATA |
|---:|---|---|
| `0x01` | 主控→模块 | 8 字节请求 |
| `0x81` | 模块→主控 | 8 字节 ACK |
| `0x90` | 模块→主控 | 1 字节页号 + 8 字节遥测页 |

### 6.2 请求 DATA

```text
Byte 0      opcode
Byte 1      sequence，0..255
Byte 2..5   value，int32，小端
Byte 6      argument
Byte 7      protocol_version，必须为 1
```

请求帧的 LEN 固定为 9，总帧长固定为 14 字节。同一 sequence 在至少 1 秒内不要用于不同请求；重试必须发送完全相同的帧。

发送“移动到 −50 mm”，sequence=1 的完整字节示例：

```text
A5 5A 09 01 10 01 B0 3C FF FF 00 01 49 42
```

发送 STOP，sequence=2：

```text
A5 5A 09 01 03 02 00 00 00 00 00 01 A3 97
```

### 6.3 ACK DATA

```text
Byte 0      opcode
Byte 1      sequence
Byte 2      result
Byte 3      flags
Byte 4..7   value，int32，小端
```

ACK 帧 LEN=9，总长 14 字节。运动和配置命令通常先返回 `result=0xFE`，表示已进入运动任务队列；执行检查完成后，再以相同 opcode/sequence 返回最终结果。只有最终 `result=0` 表示接受，是否到位还需看遥测。


无效帧、未知操作码或参数校验失败时，错误ACK的value可能为0；先按opcode/sequence关联并检查result，不应要求此类错误也回显原目标。CRC错误或长度错误的流帧可能直接丢弃而没有ACK。INFO成功ACK的value是查询结果，其他合法请求正常回显原value。

### 6.4 会话与心跳

HELLO建立会话；除HELLO、INFO和STOP外的命令都要求有效会话：

1. 主控生成非零随机 int32 nonce，发送 HELLO。
2. HELLO 成功后 flags.bit6=1。
3. 主控建议每 100 ms 发送 KEEPALIVE，value 为相同 nonce。
4. 超过 500 ms 没有有效 KEEPALIVE，模块撤销会话并停止该 UART 会话发起的运动。
5. 完成控制后发送 CLOSE，value 为相同 nonce。
6. STOP 不需要会话，可以随时发送。

不要在 ACK 超时后盲目重复新的运动 sequence。先发送 STOP、重新建立会话并读取遥测确认当前状态。

## 7. 操作码

| opcode | 名称 | value | argument | 功能 |
|---:|---|---:|---:|---|
| `0x01` | HELLO | 非零 nonce | 0 | 建立控制会话 |
| `0x02` | KEEPALIVE | nonce | 0 | 续租 |
| `0x03` | STOP | 0 | 0 | 立即请求停止，无需会话 |
| `0x04` | INFO | 0～30 | 0 | 读取配置页，无需会话 |
| `0x05` | CLOSE | nonce | 0 | 关闭会话 |
| `0x10` | MOVE_UM | −180000～+180000 | 0 | 移动到目标 µm |
| `0x11` | HOME | 0 | 0 | 移动到中心 0 mm |
| `0x12` | SET_CENTER | `0x33455453` | 0 | PB14 有效处保存中心 |
| `0x13` | ABS_CHECK | 0 | 0 | 离开中心后验证绝对方向 |
| `0x14` | ABS_READ | 0 | 0 | 刷新绝对编码器诊断值 |
| `0x16` | GOTO_POINT | 0 | 3 | 移动到保存的中心点 3 |
| `0x17` | RESTORE_REF | 0 | 0 | 从绝对计数恢复参考，不自动运动 |
| `0x18` | SPEED | 120～3000 | 0 | 设置巡航速度 rpm |
| `0x19` | BOOT_HOME | 0 或 1 | 0 | 禁用/启用上电自动回中 |
| `0x1A` | RESET | `0x52535421` | 0 | 锁闸停稳时复位MCU，保留中心；运动中拒绝 |

## 8. INFO 页

| 页 | 返回 value |
|---:|---|
| 0 | UART 协议版本，当前为 1 |
| 1 | 固件诊断版本，当前为 31 |
| 2 | 最小目标，固定 `-180000 µm` |
| 3 | 最大目标，固定 `+180000 µm` |
| 4 | absolute_stage：0 未建立，1 已存中心绝对零点，2 方向已验证 |
| 5 | 配置标志；bit3=上电自动回中 |
| 6 | 巡航速度 rpm |
| 7 | 下边界，固定 `-180000 µm` |
| 8 | 上边界，固定 `+180000 µm` |
| 9 | 保存点掩码；正常为 `0x04`，只保存中心点 3 |
| 10～14 | 点1～5高度 µm；当前只有页12的点3有效，值为0 |
| 15 | 最近绝对读取是否有效 |
| 16 | 电机型号代码 |
| 17 | 每转命令单位，期望10000 |
| 18 | 最近ABS_READ的绝对计数低32位，按无符号位模式解释 |
| 19 | 绝对计数高32位；总计数=`((page19 & 0xFFFFFFFF) << 32) + (page18 & 0xFFFFFFFF)`（Python；C中需先提升到uint64_t再移位） |
| 20 | 编码器绝对系统设置，期望1 |
| 21 | 原始多圈计数，无符号位模式 |
| 22 | 原始单圈计数，无符号位模式 |
| 23、24 | 驱动器机械计数低、高32位诊断值，不用于控制高度 |
| 25 | `ready_for_command`：1=参考有效、停稳锁闸、启动处理结束、无保护；0=尚不可发运动 |
| 26 | `reference_pending`：1=连接驱动后正在等待恢复绝对参考 |
| 27 | 内部控制状态：0启动、1配置、2就绪、3使能、4/5运动方向、6停止、7/8边界、225通信错误、226驱动错误；正常调度优先用页25与遥测 |
| 28 | 本次MCU运行期间电机通信错误累计次数；电机后上电时可非零，不等于当前仍有故障 |
| 29 | 运动保护原因：bit0参考/速度采样无效，bit1超速或减速异常，bit2过冲，bit3启动/续动稳定性异常 |
| 30 | 1=已接受RESET，等待复位；0=无待执行复位 |

## 9. 遥测

遥测帧 TYPE=`0x90`，DATA[0] 为页号 0～3，DATA[1..8] 为页内容。四页前两个字节均为 `sample_sequence uint16`。只有 sequence 相同的四页才能拼成一个样本。

| 页 | 8 字节页内容 |
|---:|---|
| 0 | sequence u16，current_um i32，speed_rpm i16 |
| 1 | sequence u16，target_um i32，target_status u8，flags u8 |
| 2 | sequence u16，fault u16，raw_mask u8，stable_mask u8，result u8，guard u8 |
| 3 | sequence u16，uptime_ms u32，feedback_age_ms u16 |

`flags`：

| 位 | 含义 |
|---:|---|
| bit0 | 相对位置有效 |
| bit1 | 目标有效 |
| bit2 | 抱闸已释放 |
| bit3 | 正在运动或有运动命令 |
| bit4 | 固定编码器范围模式标识，当前恒为1；不代表已经就绪 |
| bit5 | 已启用上电自动回中 |
| bit6 | UART 控制会话有效 |
| bit7 | 运动监控异常，运动被禁止 |

`guard`：bit0 下边界锁，bit1 上边界锁，bit2 UART RX 环形缓冲区发生过溢出，bit3 发生过帧长度/类型/CRC错误，bit4 UART 发送发生过错误。guard 错误位为本次 MCU 运行期间的粘滞诊断位。

`raw_mask/stable_mask` 仅 bit0 有效，表示 PB14 中心光耦。它们不参与正常运动停止。

`target_status`：

| 值 | 含义 |
|---:|---|
| 0 | 空闲 |
| 1 | 已排队 |
| 2 | 运动中 |
| 3 | 到位 |
| `0xE1` | 无有效位置 |
| `0xE2` | 被按键取消 |
| `0xE3` | 驱动器错误 |
| `0xE4` | 被停止命令取消 |
| `0xE5` | 超时 |
| `0xE6` | 边界或运动监控阻止 |

## 10. 结果码

| result | 含义 |
|---:|---|
| `0x00` | 成功 |
| `0x01` | 忙 |
| `0x02` | 需要建立绝对参考 |
| `0x03` | SET_CENTER 时中心光耦条件不满足 |
| `0x04` | 超范围或参数无效 |
| `0x05` | Flash 保存失败 |
| `0x06` | 驱动器/RS485 通信错误 |
| `0x07` | 位置边界阻止 |
| `0x08` | 运动超时 |
| `0x09` | 命令取消 |
| `0x0A` | 编码器比例或方向验证失败 |
| `0x0B` | 速度、减速包络、过冲或轨迹监控异常 |
| `0x80` | 请求格式或协议版本错误 |
| `0x81` | 未知操作码 |
| `0x82` | 无有效会话 |
| `0x83` | 忙或队列满 |
| `0x84` | sequence 被不同请求重复使用 |
| `0x85` | value/argument 无效 |
| `0x86` | 会话丢失 |
| `0xFE` | 已排队，等待最终结果 |



## 11. 主控推荐流程

1. 打开 UART 后先被动收取遥测，清空可能残留的半帧。
2. 用 INFO 读取页0～9，确认协议版本1、固件版本至少28、范围±180000、absolute_stage=2。
3. 检查最新完整遥测：接收时间不超过300 ms、feedback_age_ms不超过500 ms、fault=0、flags.bit0=1、flags.bit7=0。
4. 发送 HELLO，开始每100 ms发送 KEEPALIVE。
5. 发送 MOVE_UM。
6. 等待同 sequence 的最终成功 ACK。
7. 继续解析遥测，要求连续两个不同样本同时满足 target_status=3、速度绝对值≤2 rpm、当前位置与目标误差≤10 µm、反馈新鲜。
8. 连续任务需继续 KEEPALIVE，并等待抱闸锁定、运动标志清除。锁闸后再等待至少1.3秒并持续接收遥测，然后发送下一条运动命令；最后一条任务完成后发送 CLOSE。控制器有1200 ms的停稳等待条件，过早发命令会返回 BUSY，不代表通信断开。

上位机错误标签应区分：超过300 ms无完整遥测为 `uart_feedback_timeout`；UART帧仍在到达但 feedback_age_ms>500 为 `motor_feedback_stale`；flags.bit0=0 为 `position_invalid`。

### 11.1 v25 启动等待与反馈新鲜度

v25 在松闸后的300 ms机械等待期间持续读取电机位置、速度和故障；完成等待且反馈确认未漂移后才发送位置运动指令。检测到非零速度或超过20个位置指令单位（按当前比例为2 µm）的漂移，会停止并锁定运动保护；读取失败按电机通信故障处理。反馈时间戳仍为真实采样开始时间，不以遥测发送时间替代。

运动命令的最终 ACK 前还有锁闸配置阶段。此阶段串口遥测仍正常发送，但电机寄存器配置会暂时阻塞反馈扫描，`feedback_age_ms` 可能超过500 ms；这仍应显示为电机反馈暂旧，不能显示成 UART 断连。严格要求全程500 ms反馈新鲜度的主控可能在该阶段主动中止。

本工程的连续运动验收仅对这个**锁闸准备阶段**设置有限等待：命令发出不足2秒、抱闸仍锁定、速度≤2 rpm、相对发令前位置偏差≤10 µm、反馈年龄≤1500 ms，且 UART 遥测持续到达。任何条件不满足即停止；一旦松闸，立即恢复500 ms的反馈年龄上限。不得将此等待应用到运动中、故障状态或无效位置，也不得无限重发命令。参考客户端不会自动重试失败的运动指令。

### 11.2 合法端点到位与方向限位（v28）

`−180000`、`+180000` µm 都是合法目标，应该完成低速停稳并返回 REACHED。旧版本把“已经接近软件限位”与“禁止继续向外运动”共用一个停止判断，导致合法端点尚在停稳时就返回 LIMIT_BLOCKED。当前固件保留新指令的方向限位，并单独允许当前 PP 端点运动完成停稳，必须同时满足：

- 位置有效、反馈年龄不超过500 ms、当前目标恰好等于所接近的端点；
- 当前位置与端点相差不超过100个位置单位，即10 µm；
- 当次PP最高速度不超过3000 rpm，连续位置计数计算的速度不超过150 rpm，且速度计算有效。

超出这个很小的到位容差、接近端点过快、位置失效或反馈过旧，均不享有此例外，仍触发保护。到位之后必须继续等速度≤2 rpm、稳定确认以及抱闸锁定，不能仅凭高度等于目标就认为动作已完成。端点处允许向中心方向离开，禁止再向外下发目标；软件工作范围仍为±180 mm。

### 11.3 连续轨迹的速度监控

P0B-00转速反馈在本机表现出滤波延迟，位置停止后读数仍可能逐渐降至0。因此 UART 的 `speed_rpm` 继续表示驱动器报告值；不能仅因它尚未降至0就判断位置仍在移动，也不能仅看到位置不变就提前解锁下一条任务。

MCU用相邻完整扫描的P0B-07位置差与真实采样时间差计算速度（有效间隔大于0且不超过500 ms），用于减速包络和端点低速判断。同时独立检查驱动器报告速度不超过当次设定的5%容差（低速最小容差30 rpm），检查目标过冲不超过10 µm，并保留最终转速≤2 rpm及位置稳定的锁闸条件。

减速包络以 `v²=2as` 计算；监控允许0.5 mm的位置/采样余量与150 rpm速度余量，并受巡航速度容差上限约束。这些是监控容差，不改变驱动器目标和加减速参数；端点的10 µm及150 rpm到位条件单独执行。发现未按预期减速会快速停止并置运动禁止，不能靠重发目标消除保护。

## 12. Python 主控示例

仓库提供：`Projects/lifting_platform_servo/tools/lift_uart_client.py`。

安装：

```powershell
python -m pip install pyserial
```

Windows 示例（SET_CENTER/ABS_CHECK仅用于首次安装标定，日常运行跳过，不要整段盲目执行）：

```powershell
python lift_uart_client.py --port COM5 info 0
python lift_uart_client.py --port COM5 --timeout 10 monitor
python lift_uart_client.py --port COM5 set-center
python lift_uart_client.py --port COM5 move 20
python lift_uart_client.py --port COM5 abs-check
python lift_uart_client.py --port COM5 move 0
python lift_uart_client.py --port COM5 boot-home 1
python lift_uart_client.py --port COM5 move -50
python lift_uart_client.py --port COM5 move 180
python lift_uart_client.py --port COM5 stop
```

完整参考实现如下，可直接交给机器人主控开发人员：

```python
"""UART host client for lifting_platform_servo protocol v1.

The module uses 115200 8N1 on PA9/PA10. Commands are binary framed and CRC
protected; this program deliberately performs no automatic reconnect or blind
motion retry after an uncertain outcome.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from decimal import Decimal, InvalidOperation
import json
import secrets
import struct
import time
from typing import Callable

VERSION = 1
SOF = b"\xA5\x5A"
FRAME_REQUEST = 0x01
FRAME_ACK = 0x81
FRAME_TELEMETRY = 0x90
SESSION_SECONDS = 0.5
HEARTBEAT_SECONDS = 0.1
QUEUED = 0xFE
OPCODES = {
    "hello": 0x01, "keepalive": 0x02, "stop": 0x03, "info": 0x04,
    "close": 0x05, "move": 0x10, "home": 0x11, "set-center": 0x12,
    "abs-check": 0x13, "abs-read": 0x14, "goto-point": 0x16,
    "restore-ref": 0x17, "speed": 0x18, "boot-home": 0x19, "reset": 0x1A,
}
SET_CENTER_KEY = 0x33455453
RESET_KEY = 0x52535421


class LiftError(RuntimeError):
    """A command was rejected, timed out, or lost its control session."""


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode_frame(frame_type: int, data: bytes) -> bytes:
    if not 1 <= frame_type <= 255 or len(data) > 16:
        raise ValueError("invalid UART frame type or length")
    body = bytes((len(data) + 1, frame_type)) + data
    return SOF + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """Incremental parser that discards noise and CRC-corrupt frames."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(SOF[:1]) else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 3:
                break
            length = self.buffer[2]
            if not 1 <= length <= 17:
                del self.buffer[0]
                self.errors += 1
                continue
            total = length + 5
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            del self.buffer[:total]
            expected = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16_ccitt(frame[2:-2]) != expected:
                self.errors += 1
                continue
            frames.append((frame[3], frame[4:-2]))
        return frames


def millimeters_to_um(text: str) -> int:
    try:
        value = Decimal(text) * 1000
    except InvalidOperation as exc:
        raise ValueError("height must be a decimal number of millimetres") from exc
    if not value.is_finite() or value != value.to_integral_value():
        raise ValueError("height must be finite with at most 0.001 mm resolution")
    if not -180000 <= value <= 180000:
        raise ValueError("height is outside -180 to +180 mm")
    return int(value)


def encode_request(opcode: int, sequence: int, value: int = 0, argument: int = 0) -> bytes:
    if not 0 <= sequence <= 255 or not 0 <= argument <= 255:
        raise ValueError("sequence and argument must be bytes")
    if opcode not in OPCODES.values():
        raise ValueError("unknown opcode")
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("value must fit signed int32")
    if opcode == OPCODES["goto-point"]:
        valid = value == 0 and argument == 3
    elif argument:
        valid = False
    elif opcode in (OPCODES["hello"], OPCODES["keepalive"], OPCODES["close"]):
        valid = value != 0
    elif opcode == OPCODES["info"]:
        valid = 0 <= value <= 30
    elif opcode == OPCODES["move"]:
        valid = -180000 <= value <= 180000
    elif opcode == OPCODES["speed"]:
        valid = 120 <= value <= 3000
    elif opcode == OPCODES["boot-home"]:
        valid = value in (0, 1)
    elif opcode == OPCODES["reset"]:
        valid = value == RESET_KEY
    elif opcode == OPCODES["set-center"]:
        valid = value == SET_CENTER_KEY
    else:
        valid = value == 0
    if not valid:
        raise ValueError("invalid value/argument for opcode")
    return struct.pack("<BBiBB", opcode, sequence, value, argument, VERSION)


@dataclass(frozen=True)
class Ack:
    opcode: int
    sequence: int
    result: int
    flags: int
    value: int


def decode_ack(data: bytes) -> Ack:
    if len(data) != 8:
        raise ValueError("ACK payload must be exactly 8 bytes")
    return Ack(*struct.unpack("<BBBBi", data))


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    current_um: int
    speed_rpm: int
    target_um: int
    target_status: int
    flags: int
    fault: int
    raw_mask: int
    stable_mask: int
    result: int
    guard: int
    uptime_ms: int
    feedback_age_ms: int
    received_monotonic: float
    received_first_monotonic: float

    @property
    def current_mm(self) -> float:
        return self.current_um / 1000

    def feedback_age_at(self, now: float) -> float:
        return self.feedback_age_ms + max(0.0, now - self.received_first_monotonic) * 1000

    def health_state(self, now: float) -> str:
        if now - self.received_monotonic > 0.3:
            return "uart_feedback_timeout"
        if self.feedback_age_at(now) > 500:
            return "motor_feedback_stale"
        if self.fault:
            return "drive_fault"
        if not self.flags & 0x01:
            return "position_invalid"
        if self.flags & 0x80:
            return "motion_inhibited"
        return "ok"

    def reached(self, target_um: int | None, after: float, now: float) -> bool:
        return (
            self.received_monotonic >= after
            and now - self.received_monotonic <= 0.3
            and self.feedback_age_at(now) <= 250
            and self.flags & 3 == 3
            and not self.flags & (4 | 8 | 128)
            and self.fault == 0
            and self.target_status == 3
            and abs(self.speed_rpm) <= 2
            and abs(self.current_um - self.target_um) <= 10
            and (target_um is None or self.target_um == target_um)
        )


class TelemetryAssembler:
    def __init__(self):
        self.pending: dict[int, tuple[float, dict[int, bytes]]] = {}
        self.last_completed: tuple[int, int] | None = None

    def feed(self, page: int, data: bytes, now: float) -> Telemetry | None:
        self.pending = {seq: item for seq, item in self.pending.items() if now - item[0] <= 0.3}
        if page not in range(4) or len(data) != 8:
            return None
        sequence = struct.unpack_from("<H", data)[0]
        if sequence not in self.pending:
            if len(self.pending) >= 4:
                del self.pending[min(self.pending, key=lambda seq: self.pending[seq][0])]
            self.pending[sequence] = (now, {})
        first, pages = self.pending[sequence]
        if page in pages and pages[page] != data:
            del self.pending[sequence]
            return None
        pages[page] = bytes(data)
        if len(pages) != 4:
            return None
        del self.pending[sequence]
        current, speed = struct.unpack_from("<ih", pages[0], 2)
        target, status, flags = struct.unpack_from("<iBB", pages[1], 2)
        fault, raw, stable, result, guard = struct.unpack_from("<HBBBB", pages[2], 2)
        uptime, age = struct.unpack_from("<IH", pages[3], 2)
        if self.last_completed == (sequence, uptime):
            return None
        self.last_completed = (sequence, uptime)
        return Telemetry(sequence, current, speed, target, status, flags, fault,
                         raw, stable, result, guard, uptime, age, now, first)


class LiftClient:
    """Single-thread owner of a pyserial-compatible port."""

    def __init__(self, serial_port, *, clock: Callable[[], float] = time.monotonic):
        self.serial = serial_port
        self.clock = clock
        self.parser = FrameParser()
        self.assembler = TelemetryAssembler()
        self.session = False
        self.nonce = 0
        self.next_sequence = 0
        self.used_sequences: dict[int, float] = {}
        self.heartbeat: tuple[int, float] | None = None
        self.last_heartbeat = 0.0
        self.pending_request: tuple[int, int] | None = None
        self.acks: list[Ack] = []
        self.samples: list[Telemetry] = []
        self.command_started = 0.0
        self.latest: Telemetry | None = None
        self.last_device_uptime: int | None = None

    def _sequence(self) -> int:
        now = self.clock()
        for _ in range(256):
            sequence = self.next_sequence
            self.next_sequence = (sequence + 1) & 255
            if now - self.used_sequences.get(sequence, -1e30) >= 1.0:
                self.used_sequences[sequence] = now
                return sequence
        raise LiftError("sequence space exhausted")

    def _send(self, payload: bytes) -> None:
        frame = encode_frame(FRAME_REQUEST, payload)
        if self.serial.write(frame) != len(frame):
            raise LiftError("short UART write")

    def _service_heartbeat(self) -> None:
        if not self.session:
            return
        now = self.clock()
        if self.heartbeat is not None:
            if now - self.heartbeat[1] > 0.3:
                raise LiftError("heartbeat ACK lost; session abandoned")
            return
        if now - self.last_heartbeat >= HEARTBEAT_SECONDS:
            sequence = self._sequence()
            self._send(encode_request(OPCODES["keepalive"], sequence, self.nonce))
            self.heartbeat = (sequence, now)
            self.last_heartbeat = now

    def _pump(self, service_heartbeat: bool = True) -> Telemetry | None:
        if service_heartbeat:
            self._service_heartbeat()
        waiting = getattr(self.serial, "in_waiting", 0)
        chunk = self.serial.read(max(1, min(waiting or 1, 256)))
        for frame_type, data in self.parser.feed(chunk):
            if frame_type == FRAME_ACK and len(data) == 8:
                ack = decode_ack(data)
                if ack.opcode == OPCODES["keepalive"] and self.heartbeat and ack.sequence == self.heartbeat[0]:
                    if ack.result != 0 or ack.value != self.nonce or not ack.flags & 64:
                        raise LiftError(f"heartbeat rejected: 0x{ack.result:02X}")
                    self.heartbeat = None
                elif self.pending_request == (ack.opcode, ack.sequence):
                    self.acks.append(ack)
            elif frame_type == FRAME_TELEMETRY and len(data) == 9:
                sample = self.assembler.feed(data[0], data[1:], self.clock())
                if sample is not None:
                    if self.last_device_uptime is not None:
                        elapsed = (sample.uptime_ms - self.last_device_uptime) & 0xFFFFFFFF
                        if elapsed == 0:
                            continue
                        if elapsed >= 0x80000000:
                            self.last_device_uptime = None
                            self.latest = None
                            self.assembler.pending.clear()
                            raise LiftError("device restarted or telemetry moved backwards")
                    self.last_device_uptime = sample.uptime_ms
                    self.latest = sample
                    self.samples.append(sample)
        return self.samples.pop(0) if self.samples else None

    def abandon_session(self) -> None:
        owned = self.session
        self.session = False
        self.heartbeat = None
        if owned:
            try:
                self._send(encode_request(OPCODES["close"], self._sequence(), self.nonce))
            except Exception:
                pass

    def request(self, opcode: int, value: int = 0, argument: int = 0,
                *, timeout: float = 2.0,
                on_sample: Callable[[Telemetry], None] | None = None,
                on_ack: Callable[[Ack], None] | None = None) -> Ack:
        if self.pending_request is not None:
            raise LiftError("nested/concurrent requests are not supported")
        if opcode not in (OPCODES["hello"], OPCODES["stop"], OPCODES["info"]) and not self.session:
            raise LiftError("this command requires connect() first")
        sequence = self._sequence()
        payload = encode_request(opcode, sequence, value, argument)
        self.pending_request = (opcode, sequence)
        self.acks.clear()
        started = self.clock()
        last_send = started
        attempts = 1
        try:
            self._send(payload)
            while self.clock() - started < timeout:
                sample = self._pump(opcode != OPCODES["close"])
                if sample is not None and on_sample:
                    on_sample(sample)
                while self.acks:
                    ack = self.acks.pop(0)
                    if opcode != OPCODES["info"] and ack.value != value:
                        continue
                    if on_ack:
                        on_ack(ack)
                    if ack.result == QUEUED:
                        attempts = 3
                        continue
                    if ack.result != 0:
                        raise LiftError(f"opcode 0x{opcode:02X} rejected: result 0x{ack.result:02X}")
                    if opcode in (OPCODES["move"], OPCODES["home"], OPCODES["goto-point"]):
                        self.command_started = self.clock()
                        self.latest = None
                        self.assembler.pending.clear()
                    if opcode == OPCODES["reset"]:
                        self.session = False
                        self.heartbeat = None
                    return ack
                now = self.clock()
                if attempts < 3 and now - last_send >= 0.1 and now - started < 0.4:
                    self._send(payload)
                    last_send = now
                    attempts += 1
            raise LiftError("command ACK timeout; outcome unknown, do not repeat motion blindly")
        except BaseException:
            self.abandon_session()
            raise
        finally:
            self.pending_request = None
            self.acks.clear()

    def connect(self) -> Ack:
        if self.session:
            raise LiftError("already connected")
        self.nonce = secrets.randbelow((1 << 31) - 1) + 1
        ack = self.request(OPCODES["hello"], self.nonce)
        if not ack.flags & 64:
            raise LiftError("HELLO ACK did not establish a session")
        self.session = True
        self.last_heartbeat = self.clock() - HEARTBEAT_SECONDS
        self.last_device_uptime = None
        self.latest = None
        return ack

    def close(self) -> None:
        if self.session:
            try:
                self.request(OPCODES["close"], self.nonce)
            finally:
                self.session = False
                self.heartbeat = None

    def monitor(self, duration: float, on_sample: Callable[[Telemetry], None]) -> None:
        started = self.clock()
        while self.clock() - started < duration:
            sample = self._pump()
            if sample is not None:
                on_sample(sample)

    def wait_reached(self, target_um: int | None, *, timeout: float = 300.0,
                     on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        started = self.clock()
        last_sample = started
        previous = None
        try:
            while self.clock() - started < timeout:
                sample = self._pump()
                if sample is None:
                    if self.clock() - last_sample > 0.5:
                        raise LiftError("fresh complete UART telemetry lost during motion")
                    continue
                last_sample = self.clock()
                if on_sample:
                    on_sample(sample)
                if sample.fault or sample.flags & 128 or sample.target_status >= 0xE0:
                    raise LiftError(f"motion stopped/rejected: status=0x{sample.target_status:02X}, fault=0x{sample.fault:04X}")
                if not sample.flags & 64 or not sample.flags & 1:
                    raise LiftError("session or valid position lost during motion")
                if sample.reached(target_um, self.command_started, self.clock()):
                    if previous is not None:
                        elapsed = (sample.uptime_ms - previous.uptime_ms) & 0xFFFFFFFF
                        if sample.sequence != previous.sequence and 0 < elapsed <= 300:
                            return sample
                    previous = sample
                else:
                    previous = None
            raise LiftError("motion completion timeout")
        except BaseException:
            self.abandon_session()
            raise

    def read_configuration(self) -> dict[str, int]:
        names = ("protocol_version", "firmware_version", "min_um", "max_um",
                 "absolute_stage", "profile_flags", "cruise_rpm", "lower_edge_um",
                 "upper_edge_um", "saved_mask")
        return {name: self.request(OPCODES["info"], page).value
                for page, name in enumerate(names)}

    def wait_ready(self, *, timeout: float = 120.0,
                   on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        """Wait through motor-first/USB-first startup; never issue a move or rebase."""
        started = self.clock()
        while self.clock() - started < timeout:
            self.monitor(0.2, on_sample or (lambda sample: None))
            sample = self.latest
            if sample is None or self.clock() - sample.received_monotonic > 0.3:
                continue
            if sample.fault or sample.flags & 128:
                raise LiftError("drive fault or motion guard; not ready")
            if (sample.flags & 1 and not sample.flags & 12 and abs(sample.speed_rpm) <= 2
                    and sample.feedback_age_at(self.clock()) <= 500):
                ready = self.request(OPCODES["info"], 25, on_sample=on_sample)
                if ready.value == 1:
                    return self.latest or sample
        raise LiftError("device not ready before deadline; check motor power/reference")

    def move_to_mm(self, height: str, *, timeout: float = 300.0,
                   on_sample: Callable[[Telemetry], None] | None = None) -> Telemetry:
        target = millimeters_to_um(str(height))
        if not self.session:
            raise LiftError("move_to_mm requires connect() first")
        lower = self.request(OPCODES["info"], 2).value
        upper = self.request(OPCODES["info"], 3).value
        if not lower <= target <= upper:
            raise ValueError(f"target outside configured limits {lower / 1000}..{upper / 1000} mm")
        self.request(OPCODES["move"], target, on_sample=on_sample)
        return self.wait_reached(target, timeout=timeout, on_sample=on_sample)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=300)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("move").add_argument("value", help="height in mm")
    sub.add_parser("info").add_argument("value", type=int)
    sub.add_parser("goto-point").add_argument("value", type=int)
    sub.add_parser("speed").add_argument("value", type=int)
    sub.add_parser("boot-home").add_argument("value", type=int)
    for name in ("monitor", "ready", "reset", "stop", "home", "set-center", "abs-check", "abs-read", "restore-ref"):
        sub.add_parser(name)
    args = parser.parse_args(argv)
    import serial
    port = serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2)
    client = LiftClient(port)
    report = lambda sample: print(json.dumps(asdict(sample), ensure_ascii=True), flush=True)
    try:
        if args.command == "monitor":
            client.monitor(args.timeout, report)
            return 0
        if args.command == "ready":
            client.wait_ready(timeout=args.timeout, on_sample=report)
            print('{"event":"ready"}', flush=True)
            return 0
        if args.command not in ("stop", "info"):
            client.connect()
        if args.command == "move":
            client.move_to_mm(args.value, timeout=args.timeout, on_sample=report)
            return 0
        value = getattr(args, "value", 0)
        argument = 0
        if args.command == "goto-point":
            if value != 3:
                raise ValueError("only centre point 3 is supported")
            argument, value = value, 0
        elif args.command == "set-center":
            value = SET_CENTER_KEY
        elif args.command == "reset":
            value = RESET_KEY
        ack = client.request(OPCODES[args.command], value, argument)
        print(json.dumps(asdict(ack)), flush=True)
        if args.command in ("home", "goto-point"):
            client.wait_reached(0, timeout=args.timeout, on_sample=report)
        return 0
    except (LiftError, ValueError, serial.SerialException) as exc:
        print(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            client.close()
        except Exception:
            pass
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
```

## 13. 验收清单

### 接线

- PA9 接主控 RX，PA10 接主控 TX，双方共地。
- 电平为 3.3 V TTL；未接 RS232 电平。
- 机器人主控直连时，CH340 不驱动 PA10。
- PB14 只接中心光耦；PA6 接抱闸继电器。
- PB13、PB15、PA5 不作光耦输入；CANH/CANL 不用于控制。

### 标定

- SET_CENTER 时 PB14 稳定为高。
- ABS_CHECK 后 INFO页4=2。
- INFO页2/3为−180000/+180000。
- INFO页9为0x04。
- BOOT_HOME=1 后断电再上电，固件先锁闸恢复绝对位置，再自动回0。

### 通信

- 错误CRC帧不执行命令。
- INFO和STOP无需会话；其他控制命令无会话时返回0x82。
- 停止KEEPALIVE超过500 ms，flags.bit6清零，UART发起的运动停止。
- 四页遥测序号一致，刷新约10组/秒。
- 超范围目标±180000之外返回0x85或0x04，不发生运动。

### 运动

先在人员可立即急停、机械无障碍时测试 `-50 → 0 → +50 → 0`。再接近±180 mm，确认只在最终目标停稳，速度先升后降、途中不在旧的±120/±160 mm点位停车；满速时约提前62.5 mm减速。任何实际机械尺寸或传动比例变更后，都要重新测量和标定。

### 2026-09-29 v28 真机验收结果

在用户确认现场可急停、中心正确且±180 mm机械行程允许后，使用COM5、115200 8N1完成同一会话下连续20次运动，序列为：

`+20 → 0 → −20 → 0 → +100 → −100 → +160 → −160 → +180 → −180 → +180 → −180 → 0 → +180 → 0 → −180 → +180 → −180 → +50 → 0 mm`

20/20到位，包含5次360 mm全行程。MCU的运动启动次数恰好增加20，未发生隐藏的分段重启；遥测也未检测到距最终目标超过2 mm时的中途零速停车。整轮约220秒，旧分段版本同序列约361秒；例如−180→+180 mm由约30.6秒降至14.0秒（均包含准备、停稳、锁闸和下一命令前的等待）。

最高指令仍为3000 rpm，报告转速瞬时峰值3102 rpm在5%反馈容差内；20次编码器换算到位误差最大1 µm，非外部量具测得的机械精度。松闸期间电机反馈年龄最大312 ms，锁闸配置期间最大868 ms，按11.1的有限等待处理。驱动故障、通信错误计数、主机CRC错误均为0。

另完成运动中STOP后锁闸、再次HOME，以及软件复位自动回中。软件复位不等同于整机断电测试。烧录和测试没有重设中心，两个标定页SHA256保持`727d88bcccbdd8ee1c90ae4138741db10756fde88c695d5d9a4ef2f411aa4911`；运行后SWD仍可读取。交付末态0 mm、锁闸、会话关闭、COM5释放。

逐次数据与原始日志见[v28连续梯形运动与20次验收](v28连续梯形运动与20次验收_2026-09-29.md)。本节是本机本次工况结果，不替代机械传动变更后的重新验收。

### 2026-09-29 v29 上电与UART验证

电机晚于MCU上电恢复参考通过；电机上电时MCU复位从+20 mm自动回中通过；随后−20 mm、0 mm运动通过；运动中RESET返回BUSY且未重启，STOP后HOME通过。中心备份与最终读回完全一致，最终停在0 mm并锁闸。详细结果与测试边界见[新UART手册第14节](UART机器人升降平台外设接口_RDKX5.md#14-验收范围与交付记录)。

## 14. 源码位置

- `Inc/bsp_pin.h`：PA9/PA10、PB14、PA6和按键引脚。
- `Inc/lift_uart_protocol.h`、`Src/lift_uart_protocol.c`：请求、ACK、遥测、帧封装和CRC。
- `App/Tasks/lift_uart_task.c`：UART会话、心跳、命令调度和周期遥测。
- `App/Tasks/servo_jog_task.c`：绝对位置恢复、抱闸、按键和运动控制。
- `Src/lift_motion_profile.c`：固定±180 mm、单次PP目标、减速包络和越界低速恢复规划。
- `Src/lift_positions.c`：双页Flash标定与v5到v6迁移。
- `tools/lift_uart_client.py`：可运行的主控参考实现。

当前软件边界依赖绝对编码器计数、电池和中心配对。端部光耦已退出控制，因此机器人主控在每次运动前必须检查位置有效、故障码、反馈新鲜度和固定范围。
