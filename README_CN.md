# 机器人自动抓取系统 - 使用指南

## 1. 系统原理
该系统由 **树莓派 (Python 上位机)** 和 **Arduino (下位机)** 组成。

1.  **Arduino**: 运行 `Car_Volt_Feedback_24A_with_hand_gesture_and_robo_arm.ino`。
    *   负责底层电机控制（全向移动）和机械臂动作序列。
    *   通过 USB 串口接收指令：`A`(前进), `S`(停止), `rC`(旋转), `go`(抓取), `rel`(释放) 等。
    *   **注意**：代码利用了 Arduino 中原有的 `TESTMODE` 逻辑，该模式下通过 `Serialmove()` 解析串口字符串指令。

2.  **Python**: 运行 `vision.py`。
    *   **视觉识别**: 使用 OpenCV 识别红/黄/蓝色的 Block（小方块）和 Sheet（A4纸）。通过面积和长宽比区分两者。
    *   **状态机控制**:
        *   `SEARCH_BLOCK`: 旋转寻找方块。
        *   `APPROACH_BLOCK`: 视觉对准方块，靠近后执行“过量前进”（确保物体进入机械臂范围），然后停车。
        *   `GRAB`: 发送 `go` 指令，Arduino 执行 Approach -> Clip -> Rise 动作。
        *   `SEARCH_SHEET`: 带着抓到的方块，寻找同色的 A4 纸。
        *   `APPROACH_SHEET`: 对准 A4 纸并靠近。
        *   `RELEASE`: 发送 `rel` 指令放下方块，并倒车 (`B`) 离开。

## 2. 硬件连接
*   **USB 连接**: 将 Arduino 的 USB 接口连接到树莓派的 USB 口。
*   **摄像头**: 树莓派连接 USB 摄像头。
*   **电源**: 确保 Arduino 小车有独立供电（电机耗电大，仅靠 USB 供电不足）。

## 3. 测试步骤

### 第一步：确认端口
在树莓派终端运行：
```bash
ls /dev/tty*
```
找到 Arduino 的端口号（通常是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`）。
**修改代码**：打开 `vision.py`，修改第 15 行：
```python
SERIAL_PORT = '/dev/ttyUSB0' # 改为你实际的端口
```

### 第二步：运行程序
```bash
python3 vision.py
```

### 第三步：调试
1.  程序启动后，会显示摄像头画面。
2.  **State: SEARCH_BLOCK**: 机器人应该原地旋转。
3.  放入一个红色方块。机器人应该停止旋转，转向方块，并前进。
4.  **Approach**: 观察机器人是否能对准。如果方向反了（想左转却右转），请修改 `vision.py` 中 `APPROACH_BLOCK` 部分的 `rC` 和 `rA`。
5.  **Grab**: 机器人停在方块前，机械臂应该自动下降、闭合、抬起。
6.  **Release**: 随后机器人寻找红色纸张，靠近后松开。

## 4. 常见问题
*   **串口报错**: 检查 USB 线是否插好，权限是否足够 (`sudo chmod 777 /dev/ttyUSB0`)。
*   **颜色识别不准**: 环境光线影响很大。可以调整 `vision.py` 中的 `color_ranges` HSV 阈值。
*   **动作卡顿**: Arduino 的 `Serial.readStringUntil` 有 1秒 超时，如果 Python 发送指令不够快，可能会有顿挫感。Python 代码采用了 `move_and_stop` 策略来规避失控，即“动一下-停一下”的脉冲式控制，既安全又便于调试。

