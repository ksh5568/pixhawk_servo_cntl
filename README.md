# pixhawk_servo_cntl

ROS 2에서 Pixhawk(PX4)의 서보 출력을 제어하는 패키지입니다. 동일한 기능을 C++과 Python 두 가지 구현으로 제공합니다.

## 코드 아키텍처

```text
/servo/angle_cmd (Float32, -90~0 deg)
                |
                v
servo_control_node
  - control_mode 파라미터로 RC/R2 모드 선택
  - 각도 명령을 -1.0~1.0 범위로 변환
                |
                +--> /servo/mode (String: RC 또는 R2)
                +--> /servo/control (Float32: -1.0~1.0)
                              |
                              v
servo_bridge_node
  - RC 모드: PX4 RC AUX1 입력 사용
  - R2 모드: /servo/control 입력 사용
  - arm 파라미터로 PX4 ARM/DISARM 명령 전송
                              |
                              v
/fmu/in/actuator_servos -> Pixhawk servo control[0]
```

주요 파일:

- `src/servo_control_node.cpp`: 제어 모드 선택 및 각도 명령 변환
- `src/servo_bridge_node.cpp`: ROS 2 명령을 PX4 서보 메시지로 전달
- `pixhawk_servo_cntl/servo_control_node.py`: Python 제어 노드
- `pixhawk_servo_cntl/servo_bridge_node.py`: Python PX4 연결 노드
- `CMakeLists.txt`: C++ 및 Python 노드의 빌드·설치 설정

PX4 통신에는 `px4_msgs`와 PX4 uXRCE-DDS Agent가 필요합니다.
패키지는 C++17, Python 3, `ament_cmake`, `ament_cmake_python`을 사용합니다.

## Git Clone

```bash
cd ~/ros2_ws/src && git clone https://github.com/ksh5568/pixhawk_servo_cntl.git
```

## 빌드

ROS 2 워크스페이스의 `src` 아래에 패키지를 배치한 후 워크스페이스 루트에서 실행합니다.

```bash
colcon build --packages-select pixhawk_servo_cntl
source install/setup.bash
```

새 터미널을 열 때마다 `source install/setup.bash`를 다시 실행해야 합니다.

## 실행

C++ 또는 Python 중 한 가지 구현을 선택해서 실행합니다. 두 구현은 같은 노드명과 토픽을 사용하므로 동시에 실행하지 마세요.

### C++ 노드

터미널 1에서 PX4와 직접 통신하는 bridge 노드를 실행합니다.

```bash
ros2 run pixhawk_servo_cntl servo_bridge_node
```

터미널 2에서 control 노드를 실행합니다. 기본 모드는 `RC`입니다.

```bash
ros2 run pixhawk_servo_cntl servo_control_node
```

### Python 노드

터미널 1:

```bash
ros2 run pixhawk_servo_cntl servo_bridge_node_py
```

터미널 2:

```bash
ros2 run pixhawk_servo_cntl servo_control_node_py
```

R2 모드로 바로 시작하려면 선택한 구현의 control 노드에 파라미터를 전달합니다.

```bash
ros2 run pixhawk_servo_cntl servo_control_node \
  --ros-args -p control_mode:=R2
```

Python 구현은 다음과 같습니다.

```bash
ros2 run pixhawk_servo_cntl servo_control_node_py \
  --ros-args -p control_mode:=R2
```

R2 모드에서 각도 명령을 보내는 예시입니다. 입력 범위는 `-90.0~0.0`도입니다.

```bash
ros2 topic pub --once /servo/angle_cmd std_msgs/msg/Float32 "{data: -45.0}"
```

## ROS 2 파라미터 변경

현재 파라미터 확인:

```bash
ros2 param get /servo_control_node control_mode
ros2 param get /servo_bridge_node arm
```

제어 모드를 RC로 변경:

```bash
ros2 param set /servo_control_node control_mode RC
```

제어 모드를 R2로 변경:

```bash
ros2 param set /servo_control_node control_mode R2
```

PX4 ARM:

```bash
ros2 param set /servo_bridge_node arm true
```

PX4 DISARM:

```bash
ros2 param set /servo_bridge_node arm false
```

> ARM 명령은 실제 모터/서보를 움직일 수 있습니다. 기체를 안전하게 고정하고 주변을 확인한 후 실행하세요.
