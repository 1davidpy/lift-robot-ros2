# UART机器人升降平台外设接口手册 — RDK X5 USB接入

适用固件：**v31（兼容v29 UART协议）及后续兼容版本**；线协议版本：**1**；2026-09-29。

本文件面向机器人控制与训练程序开发者。只需此文档即可接线、识别设备、实现完整收发、位置控制、停止、回中、复位、中心设置和故障处理。文末附完整可运行Python客户端，复制保存即可使用，不依赖本工程其他源码。正常调用不需要配置电机寄存器、加减速度、抱闸时序或光耦。

## 1. 把平台当作一个位置外设

| 项目 | 对控制方的约定 |
|---|---|
| 接口 | RDK X5的USB Host口 → 数据线 → 本设备USB转串口芯片 |
| 串口 | 115200 bit/s，8数据位，无校验，1停止位；关闭软/硬件流控 |
| 数据 | 二进制帧+CRC，**不是发送ASCII数字或文本命令** |
| 坐标 | 已保存中心0 mm，向上为正、向下为负 |
| 目标 | −180～+180 mm；线上用有符号int32 µm，−180000～+180000 |
| 最小表示单位 | 1 µm；表示分辨率不等于机械定位精度 |
| 控制方式 | 点到点任务：一次接受一个目标，运动结束后才接下一目标 |
| 反馈 | 无需订阅，约10组/秒，包含当前位置、目标、运动/锁闸/故障等 |
| 接收确认 | 合法且入队的运动请求先给QUEUED，再给最终接受或拒绝；同opcode/sequence并回显目标；未入队可直接拒绝 |
| 加减速 | 固件内部处理；一个正常目标只启动一次，连续减速到位 |
| 主控失联 | 该UART会话发起的任务停止；不会恢复旧目标 |
| 自动回中 | 已完成绝对配对且BOOT_HOME=1时，MCU启动后等电机准备好再回0 |

**定位是任务接口，不是高速流式伺服接口。** 不能每10 ms发送一个新高度来覆盖运行中目标，忙时会拒绝，也不会把多个点无限排队。训练系统可在运动中持续收集反馈；下一目标应等待到位和READY。若需改变当前目标，先STOP、确认锁闸且READY，再发新目标。

## 2. RDK X5连接与串口识别

### 2.1 接线与供电

使用RDK X5的**USB Type-A Host口**接平台的USB转串口数据口，不使用RDK自己的系统调试串口，也不需要连接RDK 40-pin UART。设备中的USB转串口芯片已接MCU的PA9/PA10。电机/驱动器仍需要自己的主电源；USB枚举成功不表示电机已供电。编码器电池应持续连接。

模块内部接口供维修核对：PA9=MCU TX、PA10=MCU RX、PB14=中心光耦高有效、PA6=抱闸释放继电器高有效；PA2/PA3/PA4用于模块内部电机RS485。PB13/PB15/PA5不参与端部限位；CAN不用于控制。用户通过USB使用时不要再并接第二个串口发送端驱动PA10。除已有供电设计外，不额外并接两路5 V。

RDK官方说明USB Host支持USB转串口并创建`/dev/ttyUSB*`或`/dev/ttyACM*`节点：[RDK X5 USB接口说明](https://d-robotics.github.io/rdk_x_doc/en/Quick_start/hardware_introduction/rdk_x5/)。

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

### 2.2 Linux检查

```bash
sudo apt update
sudo apt install python3-serial
python3 -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/ /dev/serial/by-path/
ls -l /dev/ttyUSB* /dev/ttyACM*
```

节点名称以实际枚举为准，不固定写死`ttyUSB0`。优先使用`/dev/serial/by-id/...`；某些CH340没有唯一序列号或没有by-id链接，可固定RDK USB插口并使用`/dev/serial/by-path/...`。多个同型号设备时不要仅按VID/PID选第一个。逐个插拔核对设备路径，连接后再用INFO验证协议、固件和量程。

若无权限，先查看节点所属组；Ubuntu常见为dialout：

```bash
sudo usermod -aG dialout "$USER"
```

重新登录使组权限生效。不要同时让串口助手、minicom或另一个进程占用端口。Linux打开时可用`exclusive=True`；仅帮助排除其他也采用独占方式的程序，并非完整权限隔离。打开时关闭`xonxoff/rtscts/dsrdtr`，DTR/RTS不作为平台控制信号。[pySerial端口枚举](https://pyserial.readthedocs.io/en/latest/tools.html)、[串口API](https://pyserial.readthedocs.io/en/stable/pyserial_api.html)。

## 3. 任意上电顺序与就绪判定

主控、MCU/USB、电机是三个不同的状态，不能把“串口能打开”当作“能运动”。

| 场景 | 模块行为 | 主控行为 |
|---|---|---|
| MCU/USB先上电，电机未上电 | 保持锁闸；遥测继续，位置无效/未就绪；持续重试电机连接 | 允许INFO/HELLO/心跳；等待READY，不发送目标积压 |
| 电机稍后上电 | 设置零速保持、读取电池绝对计数、恢复保存中心；BOOT_HOME启用则回0 | 等INFO25=1及新鲜稳定遥测 |
| 电机先上电，MCU后启动 | 同样先锁闸、恢复参考、再自动回0 | USB节点出现后打开，再等待READY |
| 电机/MCU都运行，RDK程序后启动 | 打开串口/HELLO不会重设中心，也不会仅因连接主控而再次回中 | 查当前位置与READY；需要回中时发HOME |
| 运行期间电机断电或RS485断连 | 停止、锁闸、参考失效；重连后恢复参考，**不续跑旧目标，不自动重放开机回中** | 旧任务判为中断；连接恢复/READY后由任务层明确发HOME或新目标 |
| MCU复位 | UART会话丢失、uptime重新计时；按保存BOOT_HOME重新执行启动流程 | 清除旧ACK/任务关联，用新的客户端状态、nonce重新连接 |

以上自动回中以**中心已保存、absolute_stage=2、BOOT_HOME=1、编码器电池与驱动状态正常**为前提。无标定、电池/计数异常、机械位置不可信或超出恢复窗口时保持不可运动，不能为了“无论顺序”盲目寻找中心。上电期间STOP或按键干预会取消本次自动回中；后续恢复参考后可显式HOME。

正常启动算法：

1. 等待正确USB设备节点并打开串口；最多等待时间由机器人系统配置，不无限阻塞训练进程。
2. 收到CRC正确、四页一致的遥测，读取INFO0/1，要求协议1、固件≥29。
3. 读取INFO2/3/4/5/9，确认范围±180000、配对2、BOOT_HOME标志bit3及中心掩码0x04。
4. 等待INFO25=1，同时当前位置有效、无故障/运动禁止、锁闸、速度≤2 rpm、遥测/电机反馈新鲜。
5. HELLO建立会话，之后每100 ms KEEPALIVE。HELLO本身不是READY。
6. 再发目标或HOME。绝不在每次连接时发送SET_CENTER。

INFO25表示可接受新任务，不一定表示在中心；需要确认回中完成，还须target_status=3、target=0、当前位置接近0且已锁闸。一次配置状态读取与遥测不是事务快照，即使READY刚为1，命令仍可能因按键或故障被拒绝，必须检查最终ACK。

## 4. 调用约定：停止、运行、复位与反馈

### 4.1 发目标及实时确认

例如−50 mm编码为−50000 µm。主控建立本地任务ID，记录所分配sequence和目标，再发送MOVE_UM：

- `ACK result=0xFE`：模块已收到并排队。控制界面可显示“收到−50 mm，准备中”。尚不代表已开始运动。
- 同opcode/sequence的`ACK result=0`：接受执行，value回显−50000。正常启动含抱闸/驱动准备，最终ACK可能约1～2秒后返回。建议最终ACK超时3秒，等待时必须继续心跳和接收遥测。
- 其他result：显示具体拒绝原因，此请求不能当作执行成功。
- 接受以后，根据100 ms遥测更新当前高度、目标、MOVING/REACHED/故障。准备期间遥测的target可能仍为上一任务；主控把“本次请求目标”单独保存，以ACK关联，不能把旧target误认为已经接受新目标。

到位判定：最终ACK成功之后，至少两个不同sequence的新鲜完整样本，`target_status=3`、目标等于本次目标、位置有效、误差≤10 µm、速度≤2 rpm、运动bit3=0、抱闸释放bit2=0、故障=0且无禁止。随后等INFO25=1再发下一任务。单看ACK或单看当前位置相等都不够。

### 4.2 STOP

STOP无需会话，任何时候都可发。成功ACK仅表示停止请求已接收，**不代表瞬时停住**。持续接收直到速度≤2 rpm、运动标志清除、抱闸锁定；期望target_status变为取消0xE4。STOP不删除中心，停稳后可发新目标。实体急停与供电切断独立于UART。

### 4.3 HOME与RESET严格区分

- HOME=`0x11,value=0`：沿同一连续轨迹运动到保存中心，不重启MCU、不改变中心、不清空会话。
- RESET=`0x1A,value=0x52535421`：真正复位MCU，**保留Flash中心和配置**。只允许锁闸停稳、没有待执行/运行任务时；运动中返回BUSY。先STOP再确认停稳，才可RESET。
- RESET成功ACK后控制器预留至少250 ms发送时间，随后复位；电机离线时可能需等正在进行的连接尝试结束。用uptime重新开始确认确实发生复位，不能仅靠ACK判断。
- 成功接受RESET后立即停止旧会话心跳、丢弃旧任务；观察设备重新启动，重建解析器和客户端，等待READY，再用新nonce HELLO。若USB桥未掉电，端口节点可能不变。
- RESET等待窗口收到STOP会取消复位；若未观察到重启，不盲目补发复位，应先读状态。
- BOOT_HOME已启用时，RESET后会自动回中，因而RESET可能随后引起运动。它不是驱动器报警清除命令；驱动硬件报警、电池或机械问题不会因此自动解决。

### 4.4 连续采集与机器人训练

建议每条训练记录含：主控monotonic接收时间、MCU uptime_ms、sample_sequence、current_um、target_um、speed_rpm、target_status、flags、fault、result、guard、feedback_age_ms、本地任务ID/命令sequence、请求目标及ACK结果。

只拼同一sequence四页，丢弃不完整、超时、CRC错误或重复样本。原始位置采样时刻约为`uptime_ms - feedback_age_ms`（无符号32位回绕）；不要把旧位置按当前接收时间当成新样本。USB/系统调度有抖动，100 ms是标称周期，不是硬实时保证。

`speed_rpm`是电机报告的滤波转速，减速后可能滞后。固件内部已处理真实计数减速保护，控制方无需调整加减速；不要据此擅自放松到位或故障判断。采集反馈可以持续10 Hz，但运动指令仍是点到点任务。

单个I/O线程拥有串口和客户端，负责心跳、接收、发令；训练/ROS节点通过有界队列与它通信，避免多线程同时读串口或多个客户端争抢。示例SDK在request/monitor/wait_reached/wait_ready运行期间维护心跳；如果业务只sleep、不调用这些方法，心跳会中断。计算任务应放在另一线程/进程，反馈回调要快速返回。

## 5. 失联、错误与重新连接

| 现象 | 正确含义与处理 |
|---|---|
| USB节点消失/读写异常 | USB链路断开；旧任务失败，重新枚举端口，不盲目重发运动 |
| 超过300 ms没有完整新遥测 | UART反馈超时；超过500 ms运动监测必须停测/停止续租，并尽可能发STOP |
| UART仍到达但feedback_age_ms>500 | 电机反馈暂旧/RS485不可用，不能标成USB超时 |
| flags.bit0=0 | 位置无效，显示“等待电机/位置参考”，禁止运动 |
| fault非0 | 驱动器报警，记录代码并排查硬件 |
| flags.bit7=1 | 固件运动保护锁定，读INFO29；查明原因后停稳RESET重新验证 |
| result=0x01或0x83 | 忙/队列不可用；等READY，不密集重发 |
| 心跳超时或0x82/0x86 | 会话丢失；取消旧任务、读当前状态、用新nonce建立新会话 |
| uptime明显回退 | MCU重启；清空组帧、ACK、旧任务及会话状态 |

锁闸启动准备期间，寄存器配置会暂时阻塞电机反馈扫描。允许的有限例外是：发令不足2秒、抱闸仍锁定、速度≤2 rpm、距发令前位置偏差≤10 µm、反馈年龄≤1500 ms，且UART遥测持续到达。松闸后立即恢复500 ms上限；任何一项不满足都不应继续等待运动。v28真机记录中松闸年龄最大312 ms，锁闸准备最大868 ms。不能简单忽略所有motor_feedback_stale。

RDK先上电但模块尚未枚举时，应仅等待设备。RDK应用崩溃重启后，也不要自动恢复上次数据库中的未完成位置任务；先查询现态，再由任务层重新决策。重连不会改变保存中心，也不自动清除驱动硬件故障。

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



## 11. 中心与配置功能（通常只在安装调试时使用）

出厂/交付已设置中心时，控制方只查询，不再标定。INFO4=2、INFO9=4是正常配对条件。复位、正常断电和重新连接不要求再次SET_CENTER；整片擦除、更换MCU或未保留标定页的烧录可能丢失设置。

确需改变中心时：先把平台移动到真实机械中心，停稳锁闸且PB14中心光耦稳定高；HELLO并持续心跳，发送SET_CENTER及保护值0x33455453。成功才说明已保存Flash；随后检查INFO4。若为1，先移动离中心至少1 mm（通常+20 mm），停稳后ABS_CHECK，成功后INFO4应为2；再HOME确认0点，最后BOOT_HOME=1。已有同一电机方向配对时重新SET_CENTER可保持stage2。禁止在上电脚本或训练回合开始时无条件执行SET_CENTER。

SPEED=120～3000只改变巡航上限并保存，通常无需调用；加减速仍由固件控制。BOOT_HOME=0/1保存开机回中开关。RESTORE_REF尝试恢复绝对参考，本身不要求移动，也不会清除运动保护；随后要运动需另发HOME/MOVE。ABS_READ刷新原始诊断，随后读INFO15～24；这组值是调试快照，不是100 ms实时位置接口，训练用遥测current_um。

## 12. 只凭本文件运行Python

将文末代码保存为`lift_uart_client.py`。在RDK上使用Python 3.10或更新版本并安装python3-serial后，将PORT换成已经核对的设备路径：

```bash
PORT=/dev/serial/by-path/替换为实际设备路径
python3 lift_uart_client.py --port "$PORT" info 1
python3 lift_uart_client.py --port "$PORT" --timeout 120 ready
python3 lift_uart_client.py --port "$PORT" --timeout 10 monitor
python3 lift_uart_client.py --port "$PORT" move -50
python3 lift_uart_client.py --port "$PORT" home
python3 lift_uart_client.py --port "$PORT" stop
python3 lift_uart_client.py --port "$PORT" reset
python3 lift_uart_client.py --port "$PORT" --timeout 120 ready
```

每次CLI进程正常结束会关闭会话；同一任务序列或训练循环更适合导入同一个客户端。一次任务的接收、接受和连续位置反馈示例：

```python
import json
from dataclasses import asdict
import serial
from lift_uart_client import LiftClient, OPCODES, millimeters_to_um

def telemetry(sample):
    print(json.dumps({"event": "state", **asdict(sample)}), flush=True)

def acknowledgment(ack):
    event = "received" if ack.result == 0xFE else "accepted" if ack.result == 0 else "rejected"
    print(json.dumps({"event": event, **asdict(ack)}), flush=True)

port = serial.Serial("/dev/serial/by-path/替换实际路径", 115200,
                     timeout=0.02, write_timeout=0.2,
                     xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
client = LiftClient(port)
try:
    client.wait_ready(timeout=120, on_sample=telemetry)
    config = client.read_configuration()
    assert config["protocol_version"] == 1 and config["firmware_version"] >= 29
    assert config["min_um"] == -180000 and config["max_um"] == 180000
    client.connect()
    for mm in ("20", "-20", "0"):
        client.wait_ready(timeout=30, on_sample=telemetry)
        goal = millimeters_to_um(mm)
        client.request(OPCODES["move"], goal, timeout=3,
                       on_sample=telemetry, on_ack=acknowledgment)
        client.wait_reached(goal, timeout=60, on_sample=telemetry)
    client.wait_ready(timeout=30, on_sample=telemetry)
finally:
    # 通信/动作异常时不要重发旧目标；停止请求可无会话发送。
    try:
        client.request(OPCODES["stop"])
    finally:
        try:
            client.close()
        finally:
            port.close()
```

上例为顺序任务演示，不应直接在串口I/O线程执行耗时模型推理。RESET示例：调用`request(OPCODES["reset"], RESET_KEY)`获得成功后，旧客户端会停止心跳；等待检测到设备uptime重新计时，再创建新的`LiftClient(port)`并wait_ready。不要直接在同一个旧会话上继续运动。

首次枚举失败/USB重插应由外层监督进程重新打开端口；本SDK刻意不在失联后盲目自动续跑运动。重建对象会清除旧组帧和ACK缓存。0xFE不是最终完成；异常抛出时要记录result和当前反馈，由机器人上层决定恢复策略。

## 13. 完整帧示例

以下CRC由随文代码生成。建立HELLO后必须及时心跳；不要把这一串静态帧长时间逐条手工发送。运动示例须在READY状态执行。

| 用途 | 完整十六进制帧 |
|---|---|
| HELLO，nonce=123456，seq=1 | `A5 5A 09 01 01 01 40 E2 01 00 00 01 AC A3` |
| KEEPALIVE，同nonce，seq=2 | `A5 5A 09 01 02 02 40 E2 01 00 00 01 5B B3` |
| 目标−180 mm，seq=3 | `A5 5A 09 01 10 03 E0 40 FD FF 00 01 FB 29` |
| 目标+180 mm，seq=4 | `A5 5A 09 01 10 04 20 BF 02 00 00 01 E0 50` |
| HOME，seq=5 | `A5 5A 09 01 11 05 00 00 00 00 00 01 F5 3C` |
| STOP，seq=6 | `A5 5A 09 01 03 06 00 00 00 00 00 01 65 56` |
| 查询READY，seq=7 | `A5 5A 09 01 04 07 19 00 00 00 00 01 7A 7B` |
| RESET，seq=8 | `A5 5A 09 01 1A 08 21 54 53 52 00 01 85 32` |

## 14. 验收范围与交付记录

2026-09-29已烧录并运行v29。保存中心未改变；INFO4=2、INFO5=8（BOOT_HOME已启用）、INFO6=3000 rpm、目标范围±180000 µm。

| 测试 | 实测结果与边界 |
|---|---|
| MCU/USB先运行，电机断电后再上电 | 电机断电时持续遥测、位置无效、抱闸不释放；用户恢复电机电源后，无需MCU复位/SET_CENTER，自动恢复绝对参考并进入READY。此项开始位置在0附近，没有测得明显回中行程 |
| 电机已上电，MCU重新启动 | 先运动到+20 mm、确认锁闸，发送RESET；观测uptime重新计时，再自动运动回0 mm并锁闸。此项使用MCU软件复位验证启动路径，不等同于整板电源反复断开试验 |
| 回中后继续控制 | 同一验收流程继续执行−20 mm、0 mm目标，均收到QUEUED和最终OK，到位并锁闸 |
| 运动中禁止复位 | 向+20 mm运行期间发RESET，返回0x83 BUSY、uptime未重置；STOP后重新建会话，HOME到0成功 |
| 最终读取 | 当前位置0 µm、目标0 µm、速度0 rpm、target_status=3、flags=0x33（无会话、已锁闸）、READY=1、reference_pending=0、fault=0、guard=0；本次复位后电机通信错误计数0 |
| 中心持久化与SWD | 烧录前后及运动测试后两页标定数据逐字节相同；运行后SWD热连接正常 |
| 本地验证 | UART客户端14项单元测试、C帧协议、启动重试/回中/复位门控、抱闸释放、计数速度测试通过；升降项目及基础项目Debug/Release构建通过 |

v28已完成同一轮20次真实运动，包含±180 mm及5次360 mm全行程；v29保留其连续轨迹实现。本次v29验证针对上电、复位、停止和回中后的控制，没有重复执行20次全行程，也没有用软件仿真代替上述真机记录。

原始日志、标定备份、实测固件及SHA256清单保存在`test_records/2026-09-29-v29/`。这只是可追溯附件，控制方不需要这些文件即可按本手册集成。

**测试边界：** 实测主机是Windows上的USB转串口COM5，未连接RDK X5。Linux设备枚举、权限、USB拔插恢复及机器人调度需接入方在实际RDK上核验；运行途中电机断电重连不重放旧任务的分支已做本地控制逻辑测试，本轮没有另做带运动断电试验。电池有效、机械行程及中心参考正确仍是自动回中的前提。

## 15. 完整Python客户端源码

文件名：`lift_uart_client.py`；依赖：Python 3.10或更新版本、pySerial。将下列代码块完整保存，不要复制Markdown围栏。

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
