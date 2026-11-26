# LuoRov 八推进器 ROV —— STM32 固件 + 上位机

八推进器 ROV 首版控制系统：STM32F103C8T6 驱动 8 个电调推进器、MPU6050 陀螺仪、RS485 深度计，通过 RS485 双绞线与岸上电脑通信；电脑端提供上位机显示传感器状态、读取手柄（只读显示）并支持逐台调试电机。

> 本期**不含摄像头**。通信走 RS485（A/B 双绞线），供电走两根电源线（岸上 24V，舱内降压）。详细方案见 [`八推进器ROV设计方案.md`](八推进器ROV设计方案.md)。

## 目录结构

```
LuoRovStm32f103/
├─ Core/
│  ├─ Inc/
│  │  ├─ luorov_config.h      # 引脚线序 + 所有参数（改线改参数只改这里）
│  │  ├─ luorov_gpio.h        # 寄存器级 GPIO 助手
│  │  ├─ luorov_system.h      # 毫秒计时原语
│  │  ├─ motor.h / imu.h / depth.h / rs485.h / protocol.h
│  │  └─ main.h
│  └─ Src/
│     ├─ main.c               # 主循环 + 命令分发 + 看门狗
│     ├─ motor.c              # 8 路电调 PWM (TIM3+TIM4)
│     ├─ imu.c                # MPU6050 (软件 I2C)
│     ├─ depth.c              # 深度计 (Modbus RTU 主站)
│     ├─ rs485.c              # 两路 485 (USART1 岸站 / USART2 深度)
│     ├─ protocol.c           # 帧 + CRC16 + 解析
│     └─ stm32f1xx_it.c       # 中断服务
├─ host/
│  ├─ luorov_host.py          # 上位机 (tkinter + pyserial)
│  ├─ luorov_gamepad.py       # 手柄读取 (pygame, 只读显示)
│  └─ requirements.txt
├─ 线序与接线说明.md           # 引脚分配与接线
├─ 通信协议说明.md             # 帧格式与协议
└─ LuoRovStm32f103.ioc        # 原始 CubeMX 工程（见下方注意）
```

## 1. 引脚线序

详见 [`线序与接线说明.md`](线序与接线说明.md)。核心：

- 8 电调：TIM3(PA6,PA7,PB0,PB1) + TIM4(PB6,PB7,PB8,PB9)
- 岸站 485：USART1 (TX=PA9, RX=PA10, DE=PA8)
- 深度计 485：USART2 (TX=PA2, RX=PA3, DE=PA1)
- 陀螺仪 MPU6050：软件 I2C (SCL=PB10, SDA=PB11)
- LED：PC13

## 2. 编译固件

工具链：`arm-none-eabi-gcc` + CMake + Ninja。

```powershell
# 配置（首次）
cmake --preset Debug

# 编译
cmake --build --preset Debug
```

产物：`build/Debug/LuoRovStm32f103.elf`（可用 `arm-none-eabi-objcopy` 转 `.bin/.hex` 烧录，或用 STM32CubeProgrammer/ST-Link 直接烧 elf）。

> **重要**：本固件为**寄存器级手写**（不依赖 CubeMX 生成的 HAL 外设驱动，因为本仓库只保留了 GPIO/RCC 等部分 HAL 源文件，TIM/I2C/UART 的 HAL 源文件缺失）。**请勿用 CubeMX 重新生成代码**，否则会覆盖 `main.c`、`stm32f1xx_it.c` 等手写文件。

## 3. 上位机

依赖：Python 3.8+，`pyserial`；手柄读取需要 `pygame`（不装也能运行，手柄面板显示「不可用」，串口功能不受影响）。

```powershell
pip install -r host/requirements.txt
python host/luorov_host.py
```

操作：

1. 选 COM 口（USB 转 485 对应口）、波特率 115200，点「连接」。
2. 界面显示深度/压力/温度、横滚/俯仰/偏航、角速度、加速度、解锁状态。
3. 调试电机：先点「🔓 解锁」，再拖动 M1~M8 滑块（-100%~+100%）逐台测试转速；「全部停止」或「🛑 紧急停止」立即停桨。
4. 在水面点「深度置零」。
5. 「手柄」面板**只读**显示：两个摇杆的实时圆点图 + X/Y 数值、A/B/X/Y 按下状态、其他按键（原始序号）、以及「轴(原始)」一行全部轴值。**不控制电机**；首次连接需按一下手柄按键才会被识别。蓝牙（DInput）模式下按键/轴序号可能与 Xbox 不同——点「标定 A/B/X/Y」并依次按 A、B、X、Y 即可自动校正按键序号；若摇杆方向仍不对，看「轴(原始)」行核对序号再改 `luorov_gamepad.py` 顶部的 `AXIS_INDEX`。手柄到推力的映射留待后续接入。

## 4. 关键安全设计

- STM32 上电默认**未解锁**、全部停桨。
- 500ms 未收到有效指令自动停桨并失锁（上位机掉线即停）。
- 双向电调中位 1500us，含死区；正/负转速分别对应正/反转。

## 5. 待校准项（按实物调整，均在 `luorov_config.h`）

| 项 | 宏 | 说明 |
| --- | --- | --- |
| 压力单位换算 | `DEPTH_PRESS_SCALE` | 按传感器返回单位改（Pa/0.1kPa/kPa…） |
| 深度计从机地址 | `DEPTH_SLAVE_ADDR` | 默认 1 |
| 深度计波特率 | `DEPTH_BAUDRATE` | 默认 9600 |
| 深度计寄存器 | `DEPTH_REG_START`/`DEPTH_REG_COUNT` | 默认 0x0000 起 2 寄存器 |
| 电调中位/死区 | `ESC_NEUTRAL_US`/`ESC_DEADBAND_US` | 按电调标定 |
| 失联超时 | `CMD_TIMEOUT_MS` | 默认 500ms |

## 6. 依赖与硬件

- 主控：STM32F103C8T6（蓝派/Blue Pill 兼容，8MHz 晶振）
- 电调：Little Bee / BLHeli_S 双向模式，输入用舵机 PWM
- 电机：DJI 2312A ×8（配水桨）
- 深度计：RS485/Modbus RTU 压力变送器（0–200kPa 绝压类）
- 陀螺仪：MPU6050（I²C，地址 0x68）
- 485 收发器：MAX3485 等 3.3V 逻辑收发器 ×2
