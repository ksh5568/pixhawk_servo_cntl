import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rcl_interfaces.msg import SetParametersResult

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import Float32
from std_msgs.msg import String

from px4_msgs.msg import (
    ActuatorServos,
    VehicleCommand,
    VehicleStatus,
    ManualControlSetpoint,
)


class ServoBridgeNode(Node):

    def __init__(self):
        super().__init__('servo_bridge_node')

        # ============================================================
        # PX4 uXRCE-DDS QoS
        # ============================================================

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # Mode QoS
        # ============================================================

        self.mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # ROS2 Parameters
        #
        # ros2 param set /servo_bridge_node arm true
        # ros2 param set /servo_bridge_node arm false
        # ============================================================

        self.declare_parameter(
            'arm',
            False
        )

        self.arm_parameter_value = False

        # Parameter 변경 Callback
        self.add_on_set_parameters_callback(
            self.parameter_callback
        )

        # ============================================================
        # PX4 Servo Publisher
        # ============================================================

        self.servo_pub = self.create_publisher(
            ActuatorServos,
            '/fmu/in/actuator_servos',
            self.px4_qos,
        )

        # ============================================================
        # PX4 Vehicle Command Publisher
        #
        # ARM / DISARM 명령 전송
        # ============================================================

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.px4_qos,
        )

        # ============================================================
        # PX4 Vehicle Status
        # ============================================================

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            self.px4_qos,
        )

        # ============================================================
        # RC Input
        #
        # RC CH8 -> AUX1
        # ============================================================

        self.manual_control_sub = self.create_subscription(
            ManualControlSetpoint,
            '/fmu/out/manual_control_setpoint',
            self.manual_control_callback,
            self.px4_qos,
        )

        # ============================================================
        # Servo Mode
        #
        # RC
        # R2
        # ============================================================

        self.mode_sub = self.create_subscription(
            String,
            '/servo/mode',
            self.mode_callback,
            self.mode_qos,
        )

        # ============================================================
        # R2 Servo Control
        #
        # -1.0 ~ +1.0
        # ============================================================

        self.control_sub = self.create_subscription(
            Float32,
            '/servo/control',
            self.control_callback,
            10,
        )

        # ============================================================
        # Vehicle 상태
        # ============================================================

        self.nav_state = None
        self.arming_state = None

        self.ARMING_STATE_ARMED = getattr(
            VehicleStatus,
            'ARMING_STATE_ARMED',
            2,
        )

        # ============================================================
        # Mode 상태
        # ============================================================

        self.control_mode = 'HOLD'

        self.mode_received = False

        # ============================================================
        # RC 상태
        # ============================================================

        self.rc_valid = False

        self.rc_aux1 = 0.0

        self.last_rc_time = None

        self.rc_timeout = 0.5

        # ============================================================
        # R2 상태
        # ============================================================

        self.r2_valid = False

        self.r2_control = 0.0

        self.last_r2_time = None

        self.r2_timeout = 0.5

        # ============================================================
        # Servo 상태
        # ============================================================

        self.current_servo = 0.0

        self.target_servo = 0.0

        self.control_source = 'HOLD'

        # ============================================================
        # Smooth 설정
        #
        # 20 Hz
        # max_step = 0.03
        # ============================================================

        self.max_step = 0.03

        # ============================================================
        # Timer
        # ============================================================

        self.timer_period = 0.05

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
        )

        self.log_counter = 0

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'Servo Bridge Node Started'
        )

        self.get_logger().info(
            'Mode RC: RC AUX1 -> Servo'
        )

        self.get_logger().info(
            'Mode R2: /servo/control -> Servo'
        )

        self.get_logger().info(
            'ARM: ros2 param set '
            '/servo_bridge_node arm true'
        )

        self.get_logger().info(
            'DISARM: ros2 param set '
            '/servo_bridge_node arm false'
        )

        self.get_logger().info(
            'PX4 Output: '
            '/fmu/in/actuator_servos control[0]'
        )

        self.get_logger().info(
            'NO Offboard Control'
        )

        self.get_logger().info(
            '=============================================='
        )

    # ================================================================
    # Time
    # ================================================================

    def now_us(self):

        return int(
            self.get_clock().now().nanoseconds
            / 1000
        )

    def now_sec(self):

        return (
            self.get_clock().now().nanoseconds
            / 1_000_000_000.0
        )

    # ================================================================
    # Clamp
    # ================================================================

    def clamp(
        self,
        value,
    ):

        return max(
            -1.0,
            min(
                1.0,
                float(value),
            ),
        )

    # ================================================================
    # ROS2 Parameter Callback
    #
    # arm=true
    #     -> ARM
    #
    # arm=false
    #     -> DISARM
    # ================================================================

    def parameter_callback(
        self,
        params,
    ):

        for param in params:

            if param.name == 'arm':

                # bool 타입만 허용
                if (
                    param.type_
                    != Parameter.Type.BOOL
                ):

                    return SetParametersResult(
                        successful=False,
                        reason='arm parameter must be bool'
                    )

                requested_arm = bool(
                    param.value
                )

                self.arm_parameter_value = (
                    requested_arm
                )

                # --------------------------------------------
                # ARM
                # --------------------------------------------

                if requested_arm:

                    self.arm()

                    self.get_logger().info(
                        'ROS2 PARAM: ARM requested'
                    )

                # --------------------------------------------
                # DISARM
                # --------------------------------------------

                else:

                    self.disarm()

                    self.get_logger().info(
                        'ROS2 PARAM: DISARM requested'
                    )

        return SetParametersResult(
            successful=True
        )

    # ================================================================
    # PX4 Vehicle Command
    # ================================================================

    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.now_us()

        msg.param1 = float(
            param1
        )

        msg.param2 = float(
            param2
        )

        msg.param3 = float(
            param3
        )

        msg.param4 = float(
            param4
        )

        msg.param5 = float(
            param5
        )

        msg.param6 = float(
            param6
        )

        msg.param7 = float(
            param7
        )

        msg.command = int(
            command
        )

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(
            msg
        )

    # ================================================================
    # ARM
    # ================================================================

    def arm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )

        self.get_logger().info(
            'Sent PX4 ARM command'
        )

    # ================================================================
    # DISARM
    # ================================================================

    def disarm(self):

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )

        self.get_logger().info(
            'Sent PX4 DISARM command'
        )

    # ================================================================
    # Vehicle Status
    # ================================================================

    def vehicle_status_callback(
        self,
        msg: VehicleStatus,
    ):

        self.nav_state = (
            msg.nav_state
        )

        self.arming_state = (
            msg.arming_state
        )

    # ================================================================
    # RC Callback
    # ================================================================

    def manual_control_callback(
        self,
        msg: ManualControlSetpoint,
    ):

        if not msg.valid:

            self.rc_valid = False

            return

        if not math.isfinite(
            msg.aux1
        ):

            self.rc_valid = False

            return

        self.rc_aux1 = self.clamp(
            msg.aux1
        )

        self.rc_valid = True

        self.last_rc_time = (
            self.now_sec()
        )

    # ================================================================
    # Mode Callback
    # ================================================================

    def mode_callback(
        self,
        msg: String,
    ):

        requested_mode = (
            msg.data.upper()
        )

        if requested_mode not in [
            'RC',
            'R2',
        ]:

            self.get_logger().warn(
                f'Invalid servo mode: '
                f'{requested_mode}'
            )

            return

        if (
            requested_mode
            != self.control_mode
        ):

            old_mode = (
                self.control_mode
            )

            self.control_mode = (
                requested_mode
            )

            self.get_logger().info(
                f'Servo Mode Changed: '
                f'{old_mode} -> '
                f'{self.control_mode}'
            )

        self.mode_received = True

    # ================================================================
    # R2 Control Callback
    # ================================================================

    def control_callback(
        self,
        msg: Float32,
    ):

        if not math.isfinite(
            msg.data
        ):

            self.r2_valid = False

            return

        self.r2_control = (
            self.clamp(
                msg.data
            )
        )

        self.r2_valid = True

        self.last_r2_time = (
            self.now_sec()
        )

    # ================================================================
    # RC Active
    # ================================================================

    def is_rc_active(self):

        if not self.rc_valid:

            return False

        if self.last_rc_time is None:

            return False

        elapsed = (
            self.now_sec()
            - self.last_rc_time
        )

        return (
            elapsed
            <= self.rc_timeout
        )

    # ================================================================
    # R2 Active
    # ================================================================

    def is_r2_active(self):

        if not self.r2_valid:

            return False

        if self.last_r2_time is None:

            return False

        elapsed = (
            self.now_sec()
            - self.last_r2_time
        )

        return (
            elapsed
            <= self.r2_timeout
        )

    # ================================================================
    # Control Source 선택
    # ================================================================

    def select_control_source(self):

        # Mode 미수신
        if not self.mode_received:

            self.control_source = (
                'WAIT_MODE'
            )

            self.target_servo = (
                self.current_servo
            )

            return

        # ============================================================
        # RC MODE
        # ============================================================

        if self.control_mode == 'RC':

            if self.is_rc_active():

                self.control_source = (
                    'RC'
                )

                self.target_servo = (
                    self.rc_aux1
                )

                return

            self.control_source = (
                'RC_HOLD'
            )

            self.target_servo = (
                self.current_servo
            )

            return

        # ============================================================
        # R2 MODE
        # ============================================================

        if self.control_mode == 'R2':

            if self.is_r2_active():

                self.control_source = (
                    'R2'
                )

                self.target_servo = (
                    self.r2_control
                )

                return

            self.control_source = (
                'R2_HOLD'
            )

            self.target_servo = (
                self.current_servo
            )

            return

        # ============================================================
        # Unknown
        # ============================================================

        self.control_source = (
            'UNKNOWN_HOLD'
        )

        self.target_servo = (
            self.current_servo
        )

    # ================================================================
    # Smooth Servo
    # ================================================================

    def update_smooth_servo(self):

        error = (
            self.target_servo
            - self.current_servo
        )

        if abs(error) <= self.max_step:

            self.current_servo = (
                self.target_servo
            )

        elif error > 0.0:

            self.current_servo += (
                self.max_step
            )

        else:

            self.current_servo -= (
                self.max_step
            )

        self.current_servo = (
            self.clamp(
                self.current_servo
            )
        )

    # ================================================================
    # PX4 Servo Publish
    # ================================================================

    def publish_servo_command(self):

        self.update_smooth_servo()

        msg = ActuatorServos()

        now = self.now_us()

        msg.timestamp = now
        msg.timestamp_sample = now

        msg.control = [
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
            float('nan'),
        ]

        # Servo 1
        msg.control[0] = float(
            self.current_servo
        )

        self.servo_pub.publish(
            msg
        )

    # ================================================================
    # Timer
    # ================================================================

    def timer_callback(self):

        # RC / R2 선택
        self.select_control_source()

        # Servo 명령
        self.publish_servo_command()

        # 상태 로그
        if self.log_counter % 20 == 0:

            armed = (
                self.arming_state
                == self.ARMING_STATE_ARMED
            )

            self.get_logger().info(
                f'mode={self.control_mode}, '
                f'source={self.control_source}, '
                f'nav_state={self.nav_state}, '
                f'arming_state={self.arming_state}, '
                f'armed={armed}, '
                f'arm_param={self.arm_parameter_value}, '
                f'rc_active={self.is_rc_active()}, '
                f'rc_aux1={self.rc_aux1:.3f}, '
                f'r2_active={self.is_r2_active()}, '
                f'r2_control={self.r2_control:.3f}, '
                f'target={self.target_servo:.3f}, '
                f'servo={self.current_servo:.3f}'
            )

        self.log_counter += 1


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ServoBridgeNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Servo Bridge Node stopped'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
