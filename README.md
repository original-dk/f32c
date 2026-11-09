# F32C Python 串口驱动

[English](README_EN.md)

基于 WHEELTEC F32C 的 STM32F1 位置闭环、速度闭环例程及 TI 云台例程整理。电脑通过 **USB 转 TTL 串口模块**直接控制电机，不需要 STM32、Keil 或 ST-Link。

本项目包含通用串口驱动、双轴云台示例、角度标定脚本和自动测试。示例默认使用：

- 地址 `0x01`：下方 yaw 电机；
- 地址 `0x02`：上方 pitch 电机；
- 多圈位置直通模式 `Mode.MULTI_POSITION_DIRECT`（`0x0003`）。

> **安全提示：**自动测试不能替代实机验证。首次使用请先运行只读示例，再进行小角度、低速度测试。运动前固定云台并妥善支撑相机等负载；失能或断电后，机构可能因重力或线缆拉力滑动。

## 1. 功能概览

- 电机使能、失能及模式设置；
- 速度控制，单位 RPM；
- 多圈绝对位置控制：有符号 32 位原始值，分辨率 0.1°，允许负角度和累计圈数；
- 单圈绝对位置控制：0.0～359.9°，分辨率 0.1°；
- 加速度设置；
- 速度、多圈角度、单圈角度、加速度和电压反馈；
- 保存参数、多圈角度清零、单圈零点设置、地址修改和恢复出厂设置命令；
- BCC 校验、噪声丢弃、错误帧重新同步及任意串口分片接收；
- 同一总线上的多电机地址支持和反馈超时处理；
- 双轴周期目标重发、首次到位检测、到位后保持及退出失能处理。

## 2. 项目结构

以下结构与本源码包一致：

```text
f32c-master/
├─ f32c/
│  ├─ __init__.py              对外导出驱动 API
│  ├─ protocol.py             命令帧、BCC、模式及反馈解析
│  └─ driver.py               pyserial 总线与电机 API
├─ examples/
│  ├─ list_ports.py           列出电脑当前可见的串口
│  ├─ read_angles.py          只读多圈角度和电压，不使能电机
│  ├─ calibration.py          读取两轴角度并写入标定文件
│  ├─ two_axis_start_pos.py   以本次启动时实际角度为基准的相对运动
│  └─ two_axis_abs_pos.py     以标定文件中的角度为参考基准的运动
├─ tests/
│  ├─ test_protocol.py        协议、已知帧、分片及错误恢复测试
│  ├─ test_driver.py          虚拟串口驱动测试
│  └─ test_hold_state.py      到位后保持状态的回归测试
├─ calibration.json           两轴标定参考角度
├─ pyproject.toml             Python 项目与依赖配置
├─ requirements.txt           pyserial 依赖
├─ README.md                  中文说明
└─ README_EN.md               英文说明
```

## 3. 接线与串口参数

```text
电脑 USB
   │
USB 转 TTL 模块                 F32C 电机总线
   TX  ───────────────────────→  电机 RX
   RX  ←───────────────────────  电机 TX
   GND ─────────────────────────  GND
                                  │
                         地址 0x01：yaw 电机
                         地址 0x02：pitch 电机
```

串口参数为 **115200、8 数据位、无校验、1 停止位、无流控**，即 **115200 8N1**。

注意事项：

1. USB-TTL 的 TX/RX 必须与电机交叉连接，并确保电机、电机电源和 USB-TTL 共地。
2. 使用 TTL 串口，不要直接连接传统 RS-232 电平接口；TTL 电平和电机供电要求以电机手册为准。
3. 不要使用 USB-TTL 模块给电机供电，不要热插拔。
4. 两台电机共用总线时必须使用不同地址；示例默认地址为 1 和 2。
5. 首次测试应卸载或支撑负载，并确保发生异常时可以切断电机电源。

## 4. 安装

需要 **Python 3.9 或更高版本**，唯一外部依赖为 `pyserial`。建议先在项目根目录创建并激活虚拟环境。

### Windows

```bat
py -m pip install -e .
py examples\list_ports.py
```

### Linux / macOS

```bash
python3 -m pip install -e .
python3 examples/list_ports.py
```

如果已经激活虚拟环境并使用 `uv`，也可以将安装命令替换为：

```bash
uv pip install -e .
```

不使用可编辑安装时，可执行 `python -m pip install .`。

## 5. 推荐的上电检查顺序

### 5.1 查找串口

```bash
python examples/list_ports.py
```

Windows 串口通常类似 `COM8`，Linux 通常类似 `/dev/ttyUSB0`。

### 5.2 只读角度和电压

将 `COM8` 替换为实际串口：

```bash
python examples/read_angles.py --port COM8 --address 1 2
```

该程序只请求多圈角度和电压，**不会使能电机，也不会发送位置或速度命令**。若反馈超时，请依次检查：

1. 串口是否选错；
2. TX/RX 是否交叉连接；
3. 是否共地；
4. 电机是否已上电；
5. 电机地址是否为 1、2；
6. 波特率是否为 115200。

## 6. 标定参考角度

`examples/calibration.py` 用于把当前两轴的多圈角度记录为参考值，适用于“先手动摆到指定姿态，再保存该姿态角度”的场景。

### 6.1 使用方法

1. 给电机上电，手动将 yaw、pitch 摆到需要记录的姿态，并可靠支撑负载。
2. 打开 `examples/calibration.py`，把其中的 `F32CBus("COM8")` 改为实际串口。该脚本当前没有 `--port` 命令行参数。
3. 在**项目根目录**运行：

   ```bash
   python examples/calibration.py
   ```

4. 脚本会在当前工作目录创建或覆盖 `calibration.json`：

   ```json
   {
     "yaw_init": -2970.7,
     "pitch_init": -2680.6
   }
   ```

   上述数值只是当前包内的示例，不同设备、装配和姿态不能直接复用。

### 6.2 标定脚本实际做了什么

- 读取地址 1、2 的多圈角度；
- 将结果写入 `yaw_init`、`pitch_init`；
- 不使能电机，不发送运动命令；
- 不调用多圈清零、单圈置零或参数保存命令；
- 不会自动验证姿态是否垂直，姿态需要由使用者通过机械基准、水平仪或其他方式确认。

`two_axis_abs_pos.py` 同样从当前工作目录读取 `calibration.json`，因此标定和运行该示例时都应从项目根目录启动，避免读写错文件。

## 7. 双轴运动示例

两个运动示例都要求显式添加 `--confirm`，否则程序拒绝使能电机。两者均使用多圈位置直通模式、周期重发目标，并在退出时尝试失能两轴。

### 7.1 以本次启动位置为基准：`two_axis_start_pos.py`

该示例不使用 `calibration.json`。程序启动后先读取两轴的实际多圈角度，再叠加命令行增量：

```python
yaw_start = yaw.read_angle()
pitch_start = pitch.read_angle()
yaw_target = yaw_start + args.yaw_delta
pitch_target = pitch_start + args.pitch_delta
```

确认机械安全后，下面的命令让 yaw 相对当前位置增加 10°、pitch 减少 5°：

```bash
python examples/two_axis_start_pos.py --port COM8 --yaw-delta 10 --pitch-delta -5 --speed 10 --hold-time 10 --confirm
```

只检查 pitch 时，可令 yaw 增量为 0：

```bash
python examples/two_axis_start_pos.py --port COM8 --yaw-delta 0 --pitch-delta -2 --speed 5 --hold-time 10 --confirm
```

这里的 `--yaw-delta 0` 表示 yaw 的目标是其**本次启动时读到的位置**，不是电机坐标系的 0°。

### 7.2 以标定参考位置为基准：`two_axis_abs_pos.py`

该示例读取 `calibration.json`，但 `--yaw-delta`、`--pitch-delta` 仍然是**相对标定参考角度的增量**，不是直接输入的绝对目标角度。当前源码的计算方式为：

```python
yaw_start = data["yaw_init"]
pitch_start = data["pitch_init"]
yaw_target = yaw_start + args.yaw_delta
pitch_target = pitch_start + args.pitch_delta
```

因此，两个增量都设为 0 时，目标就是标定文件中保存的两轴参考角度：

```bash
python examples/two_axis_abs_pos.py --port COM8 --yaw-delta 0 --pitch-delta 0 --speed 10 --hold-time 0 --confirm
```

这可用于尝试回到已标定姿态并持续保持，但必须注意：

- 该脚本当前把标定值作为 `yaw_start`、`pitch_start`，**没有先读取实际启动角度来计算目标**；
- 当前实现没有启动角度差保护，也没有平滑插值；目标与实际位置相差较大时，可能直接产生大幅运动；
- 运行前应先用 `read_angles.py` 检查实际角度，确认目标运动范围安全，再添加 `--confirm`；
- 固定姿态回位只有在同一物理姿态的多圈角度跨断电重启仍可重复时才可靠。若出现整圈偏移、机械打滑或底座姿态变化，仅靠 JSON 中保存的角度无法保证回到同一空间方向，需要机械回零、零位开关或外部姿态传感器等方案。

### 7.3 两个示例的区别

| 项目 | `two_axis_start_pos.py` | `two_axis_abs_pos.py` |
|---|---|---|
| 是否读取 `calibration.json` | 否 | 是 |
| 基准角度来源 | 本次启动时读取的实际多圈角度 | 文件中的 `yaw_init`、`pitch_init` |
| `--yaw-delta` / `--pitch-delta` | 相对本次启动位置的增量 | 相对标定参考角度的增量 |
| 是否适合每次从当前姿态运动 | 是 | 取决于当前姿态与标定目标的偏差 |
| 增量全为 0 时的目标 | 保持本次启动位置 | 标定文件中的参考位置 |

## 8. 双轴示例参数

以下参数同时适用于 `two_axis_start_pos.py` 和 `two_axis_abs_pos.py`：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--port` | 无，必填 | 串口名称，例如 `COM8` 或 `/dev/ttyUSB0` |
| `--yaw-address` | `1` | yaw 电机地址 |
| `--pitch-address` | `2` | pitch 电机地址；不能与 yaw 地址相同 |
| `--yaw-delta` | `0.0` | yaw 角度增量，单位为度；基准取决于所用示例 |
| `--pitch-delta` | `0.0` | pitch 角度增量，单位为度；基准取决于所用示例 |
| `--speed` | `20` | 位置模式运行速度，单位 RPM，必须大于 0 |
| `--tolerance` | `0.3` | 两轴到位判定容差，单位为度，必须大于 0 |
| `--timeout` | `10.0` | 最长等待**首次到位**的时间，单位为秒 |
| `--startup-delay` | `1.5` | 发送唤醒字节后的启动等待时间，单位为秒 |
| `--hold-time` | `5.0` | 首次到位后的保持秒数；`0` 表示持续保持到 Ctrl+C |
| `--command-period` | `0.02` | 循环中的目标重发间隔，单位为秒 |
| `--confirm` | 关闭 | 人工确认接线和机械安全；不提供则拒绝运动 |

首次实机测试建议进一步减小角度变化和速度，不要直接以大角度、高速度运行。

## 9. 到位、保持与退出行为

两个示例都使用 `Mode.MULTI_POSITION_DIRECT`（`0x0003`），并按以下顺序工作：

1. 发送 `wake()` 使用的唤醒字节并等待启动；
2. 确定目标角度；
3. 分别调用 `enable()`、设置直通模式和速度；
4. 先发送一次起始基准角度，再进入周期目标发送与反馈检查；
5. 两轴都首次进入容差后，记录首次到位时间并进入保持阶段。

保持状态具有以下行为：

- `--timeout` 只限制首次到位前的等待；首次到位后，反馈波动不会重新启动该超时；
- “首次到位”消息只打印一次；
- 有限保持时间从首次到位时开始计算；
- `--hold-time 0` 表示程序运行期间一直保持，直到 Ctrl+C，不代表退出后仍保持。

**退出行为必须特别注意：**正常保持结束、首次到位超时、Ctrl+C 或其他异常都会进入 `finally`，程序会尝试失能两台电机。失能命令发送失败时会打印错误，但程序无法保证电机一定已失能。

因此，长期保持时必须让程序持续运行；在按 Ctrl+C、关闭终端或切断电源之前，应先支撑相机和云台。**失能不是机械刹车，周期调用 `set_angle()` 也不是使能的替代品。**

## 10. 在 Python 程序中调用

### 10.1 只读反馈

```python
from f32c import F32CBus

with F32CBus("COM8") as bus:
    yaw = bus.motor(1)
    pitch = bus.motor(2)
    print("yaw:", yaw.read_angle(), "°")
    print("pitch:", pitch.read_angle(), "°")
    print("电压:", yaw.read_voltage(), "V")
```

### 10.2 多圈位置控制

```python
from f32c import F32CBus, Mode

with F32CBus("COM8") as bus:
    motor = bus.motor(1)
    try:
        motor.enable()
        motor.set_mode(Mode.MULTI_POSITION)
        motor.set_speed(10)
        current = motor.read_angle()
        motor.set_angle(current + 5.0)  # 绝对目标 = 当前角度 + 5°
        # 实际应用应在这里等待到位、处理异常并按需继续保持目标。
    finally:
        motor.disable()               # 失能前应先支撑负载
```

`F32CMotor.set_angle()` 接收的是**多圈绝对目标角度**，不是相对增量，也不是单圈角度。相对运动需要先读取当前角度，再计算 `current + delta`。

### 10.3 `enable()` 与 `set_angle()` 的职责

两者完全独立：

- `enable()` 发送命令 `0x06`，用于使能电机；
- `set_angle()` 通过总线调用 `set_multi_angle()`，发送命令 `0x02`，只更新多圈绝对目标；
- `set_angle()` 不会隐式使能电机；
- 周期调用 `set_angle()` 是重复更新或维持目标，不是反复使能；
- `wake()` 只发送例程使用的 `0x00` 唤醒字节，也不等同于使能。

### 10.4 常用 API

| 对象 | 方法 | 说明 |
|---|---|---|
| `F32CBus` | `motor(address)` | 创建绑定地址的 `F32CMotor` |
| `F32CBus` | `enable(address)` / `disable(address)` | 使能或失能指定地址电机 |
| `F32CBus` | `set_mode(address, mode)` | 设置运行模式 |
| `F32CBus` | `set_speed_rpm(address, rpm)` | 设置速度，单位 RPM |
| `F32CBus` | `set_multi_angle(address, angle_deg)` | 设置多圈绝对目标角度 |
| `F32CBus` | `set_single_angle(address, angle_deg)` | 设置 0.0～359.9° 的单圈目标 |
| `F32CBus` | `set_acceleration(address, rev_per_s2)` | 设置加速度 |
| `F32CBus` | `read_multi_angle(address)` | 读取多圈角度，单位为度 |
| `F32CBus` | `read_single_angle(address)` | 读取单圈角度，单位为度 |
| `F32CBus` | `read_speed_rpm(address)` | 读取速度，单位 RPM |
| `F32CBus` | `read_acceleration(address)` | 读取加速度反馈 |
| `F32CBus` | `read_voltage(address)` | 读取电压，单位 V |
| `F32CMotor` | `set_speed(rpm)` | 对绑定地址设置速度 |
| `F32CMotor` | `set_angle(angle_deg)` | 对绑定地址设置多圈绝对目标 |
| `F32CMotor` | `read_angle()` | 读取绑定地址的多圈角度 |
| `F32CMotor` | `read_speed()` / `read_voltage()` | 读取绑定地址的速度或电压 |

清零、置零、修改地址、保存参数和恢复出厂设置会改变电机状态或参数，不应在不了解当前配置时调用。普通运动流程不会自动调用这些命令。

## 11. 模式与角度约定

### 11.1 模式值

| 模式 | 枚举 | 模式值 | 说明 |
|---|---|---:|---|
| 速度 | `Mode.SPEED` | `0x0000` | 速度闭环 |
| 多圈位置规划 | `Mode.MULTI_POSITION` | `0x0001` | 电机内部带 T 型轨迹规划 |
| 单圈位置规划 | `Mode.SINGLE_POSITION` | `0x0002` | 电机内部带 T 型轨迹规划 |
| 多圈位置直通 | `Mode.MULTI_POSITION_DIRECT` | `0x0003` | 上位机周期给定多圈位置目标 |
| 单圈位置直通 | `Mode.SINGLE_POSITION_DIRECT` | `0x0004` | 上位机周期给定单圈位置目标 |

“规划”和“直通”描述的是轨迹主要由电机内部还是上位机生成；两者仍属于位置控制。**直通不是开环 PWM 控制。**

### 11.2 多圈与单圈

- 多圈角度允许负数和累计圈数，适合记录多圈旋转、标定参考角度以及本项目的双轴示例。
- 单圈角度范围为 0.0～359.9°，跨越 `0°/360°` 时不能直接把所有误差都按普通减法处理。
- 单圈圆周误差可使用：

  ```python
  def angle_error(target, current):
      return (target - current + 180.0) % 360.0 - 180.0
  ```

- 总线 API 提供 `set_single_angle()`，但 `F32CMotor.set_angle()` 固定发送多圈位置命令。
- 保存标定参考角度不等于清零电机坐标；`calibration.py` 也不会修改电机内部零点。

## 12. 协议摘要

命令帧格式：

```text
0x7A | 电机地址 | 命令字 | 数据区（可为空） | BCC | 0x7B
```

BCC 是从帧头 `0x7A` 到数据区末尾所有字节的逐字节异或，多字节整数均为大端序。

| 功能 | 命令字 | 数据区 |
|---|---:|---|
| 设置模式 | `0x00` | 2 字节无符号模式值 |
| 设置速度 | `0x01` | 2 字节有符号 RPM |
| 多圈位置 | `0x02` | 4 字节有符号角度，单位 0.1° |
| 单圈位置 | `0x03` | 2 字节无符号角度，单位 0.1° |
| 失能 | `0x05` | 无 |
| 使能 | `0x06` | 无 |
| 设置加速度 | `0x07` | 2 字节，转/s² |
| 保存参数 | `0x08` | 无 |
| 多圈角度清零 | `0x09` | 无 |
| 单圈绝对角度置零 | `0x0A` | 无 |
| 恢复出厂设置 | `0x0B` | 无 |
| 设置地址 | `0x0D` | 1 字节新地址 |
| 请求反馈 | `0x0E` | 1 字节反馈类型 |

反馈类型为：速度 `0x00`、多圈角度 `0x01`、单圈角度 `0x02`、加速度 `0x03`、电压 `0x04`。反馈帧固定为 9 字节，数据区为 4 字节；按所提供例程：

- 速度、加速度取低 16 位有符号值；
- 单圈角度、电压取低 16 位无符号值；
- 多圈角度取完整 32 位有符号值；
- 角度换算单位为 0.1°，电压换算单位为 0.01 V。

## 13. 自动测试与当前核对状态

在项目根目录运行：

```bash
python -m unittest discover -s tests -v
```

测试使用已知帧和虚拟串口，不需要真实电机。

本次对当前源码包的核对结果：

- `test_protocol.py` 中 7 项协议测试通过；
- `test_driver.py` 中 6 项虚拟串口驱动测试通过；
- `tests/test_hold_state.py` 的导入路径仍指向一个已不存在的旧双轴示例文件，因此完整测试发现流程会在导入该测试文件时报错，当前不能表述为“全部测试通过”。

如需单独复核前两部分，可运行：

```bash
python -m unittest discover -s tests -p "test_protocol.py" -v
python -m unittest discover -s tests -p "test_driver.py" -v
```

保持状态测试恢复前，需要先把 `test_hold_state.py` 中的示例路径改为实际要验证的文件，例如 `examples/two_axis_start_pos.py`，再重新运行完整测试。

## 14. 例程核对说明

- 使能帧、反馈请求帧已与 STM32F1 例程中的固定字节数组逐字节核对。
- 命令字、失能、清零、地址及完整反馈类型来自 TI 例程中的 `DataScope_DP.C/.h`。
- STM32F1 速度闭环示例的模式数组下标写法与位置闭环及 TI 通用协议实现不一致；本驱动采用后者，即 `0x00` 命令携带 2 字节模式值。
- PDF 提到的 `0x11/0x12` 位置环 PI 参数命令，在这三个通用协议实现中没有完整代码可供交叉核对，因此本版本未封装。底层 `send_command()` 可在确认协议后用于扩展。
