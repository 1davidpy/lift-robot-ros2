# 配置2：七舵机限位与遥控

当前版本使用config/config2.json，已允许1/2/3/8号限位内递增位置控制，
4号回转1000、5/6号履带3000。遥控位置速度250/500/200/200，单次最多10；Qt手动速度100、单次最多10。
完整坐标、模式准备、边界与操作流程见[config2_control.md](config2_control.md)。

配置2限位：ID1=1001..2950，ID2=2223..4076，ID3=1547..3609，ID8=172..1292。
数字使用用户原标定参考坐标；配置记录参考模式及85编码器偏移，按硬件读回校验后换算。
CMake编译到Flash，启动直接加载且不移动。历史machines.json不再作为当前编译源。
Qt默认同一个config2.json，保存保留参考模式/偏移；文件保存不自动下发或烧录。

遥控LY→2、RY→1、RX→8、BL/BR→3、LX→4。BL减、BR增，两键一起为俯仰停止，
并同时后退；不设按键调速。L1左转、R1右转、L1+R1前进。保持摇杆较大位移轴优先。
无线v2为15字节，七通道，配套更新两端；拒绝旧v1帧。

## 安全行为

上电先读反馈；2026-10-21起在中立且姿态稳定约1秒后自主准备未带力矩舵机，锁定实时位置，不回中、不追旧目标，SYNC_WRITE禁止。
显式PREPARE_REMOTE要求先支撑位置机构、履带架空、回转及线缆安全、遥控中立。
卸力后设置各轴模式，等待反馈稳定、核对参考位置不变，以新读数写目标并验证后恢复力矩。
模式/偏移/目标不正确时禁止允许状态。中立1秒后才控制。
松手保持实时位置，失联约200ms停止，再次中立1秒才能重新允许。
越界只允许小步向内回退，不直接跳到限位；实际机械端点和载荷仍需人工确认。
遥控四轴每20ms目标递增5/10/4/4，距新读数最大96；标定步长10、速度100的驱动接口单独保留。
允许小步进入安全范围，避免卡在限位外的死区。

Qt CAL_PREPARE暂停整个遥控。MOVE限速100、步长10，不叠加未完成目标；
界外同样只允许向内。LIMIT只能缩小Flash安全范围，CLEAR恢复Flash默认值。
标定后RESUME_REMOTE恢复控制，准备失败不会自动重试。

## 串口

115200 8N1，ASCII以LF结束。

```text
PING
STATUS
READINGS
MODES
FLASH_LIMITS
POSITION_DIAG <axis:0|1|2|3|4>
POSITION_DEADBAND <axis:0|1|2|3|4>
SERVO_REG <axis:0|1|2|3|4> <address:0..254>
REMOTE_INPUT
CHASSIS
BUS_DIAG
PREPARE_REMOTE
PAUSE_REMOTE
RESUME_REMOTE
CAL_PREPARE <axis:0|1|2|4>
LIMIT <axis> <minimum> <maximum>
CLEAR <axis>
MOVE <axis> <reference_position> <speed:1..100>
STOP <axis:0..4>
WHEEL_STOP_ALL
```

STATUS2沿用七舵机格式，online_mask按ID位，全在线191；limit_mask按轴0/1/2/4，全启用23。
周期CAL_STATE给手动准备位，REMOTE_STATE给enabled/armed/ready_mask/recovery_mask。
READINGS依次ID1/2/3/4/5/6/8，使用参考坐标。POSITION_DIAG给原始坐标；
FLASH_LIMITS给永久Flash值而非RAM覆盖。CHASSIS目标速度不等于实测速度。
额外JOG及非零串口底盘命令仍拒绝，底盘由遥控控制。
实机和边界验证记录见hardware_test_2026-10-14.md。
