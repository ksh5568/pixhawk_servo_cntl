import math

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import Float32
from std_msgs.msg import String


class ServoControlNode(Node):

    def __init__(self):
        super().__init__('servo_control_node')

        # ============================================================
        # Parameter
        #
        # 실행 시:
        #
        # -p control_mode:=RC
        #
        # 또는
        #
        # -p control_mode:=R2
        # ============================================================

        self.declare_parameter(
            'control_mode',
            'RC'
        )

        # ============================================================
        # Servo Angle Range
        #
        #   0 deg  -> +1.0
        # -45 deg  ->  0.0
        # -90 deg  -> -1.0
        # ============================================================

        self.angle_min = -90.0
        self.angle_max = 0.0

        # ============================================================
        # 상태
        # ============================================================

        self.current_mode = 'RC'

        self.angle_deg = 0.0

        self.normalized_control = 0.0

        # 실제 각도 명령을 한 번이라도 받았는지
        self.angle_command_received = False

        # ============================================================
        # Mode Topic QoS
        #
        # servo_bridge_node가 늦게 실행되어도
        # 마지막 mode를 받을 수 있도록 TRANSIENT_LOCAL 사용
        # ============================================================

        self.mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # Subscriber
        #
        # 알고리즘에서 원하는 각도
        #
        # -90.0 ~ 0.0 degree
        # ============================================================

        self.angle_sub = self.create_subscription(
            Float32,
            '/servo/angle_cmd',
            self.angle_callback,
            10,
        )

        # ============================================================
        # Publisher
        #
        # Servo Mode
        #
        # "RC"
        # 또는
        # "R2"
        # ============================================================

        self.mode_pub = self.create_publisher(
            String,
            '/servo/mode',
            self.mode_qos,
        )

        # ============================================================
        # Publisher
        #
        # Normalized Servo Control
        #
        # -1.0 ~ +1.0
        # ============================================================

        self.control_pub = self.create_publisher(
            Float32,
            '/servo/control',
            10,
        )

        # ============================================================
        # 20 Hz Timer
        # ============================================================

        self.timer_period = 0.05

        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback,
        )

        self.log_counter = 0

        # 최초 mode 확인
        self.update_mode()

        self.get_logger().info(
            '=============================================='
        )

        self.get_logger().info(
            'Servo Control Node Started'
        )

        self.get_logger().info(
            f'Initial Mode: {self.current_mode}'
        )

        self.get_logger().info(
            'Mode Output: /servo/mode'
        )

        self.get_logger().info(
            'Angle Input: /servo/angle_cmd [-90 ~ 0 deg]'
        )

        self.get_logger().info(
            'Servo Output: /servo/control [-1 ~ +1]'
        )

        self.get_logger().info(
            '=============================================='
        )

    # ================================================================
    # Clamp
    # ================================================================

    def clamp(
        self,
        value,
        minimum,
        maximum,
    ):

        return max(
            minimum,
            min(
                maximum,
                float(value),
            ),
        )

    # ================================================================
    # Mode Parameter 확인
    #
    # ros2 param set으로 실행 중에도 변경 가능
    # ================================================================

    def update_mode(self):

        requested_mode = str(
            self.get_parameter(
                'control_mode'
            ).value
        ).upper()

        # 허용 모드
        if requested_mode not in [
            'RC',
            'R2',
        ]:

            self.get_logger().warn(
                f'Invalid control_mode: '
                f'{requested_mode}. '
                f'Use RC or R2.'
            )

            return

        # Mode 변경 감지
        if requested_mode != self.current_mode:

            old_mode = self.current_mode

            self.current_mode = (
                requested_mode
            )

            self.get_logger().info(
                f'Servo Mode Changed: '
                f'{old_mode} -> '
                f'{self.current_mode}'
            )

        else:

            self.current_mode = (
                requested_mode
            )

    # ================================================================
    # Angle -> Normalized
    #
    # -90 deg -> -1.0
    # -45 deg ->  0.0
    #   0 deg -> +1.0
    # ================================================================

    def angle_to_normalized(
        self,
        angle_deg,
    ):

        # 입력 제한
        angle_deg = self.clamp(
            angle_deg,
            self.angle_min,
            self.angle_max,
        )

        # Linear Mapping
        normalized = (
            2.0
            * (
                angle_deg
                - self.angle_min
            )
            / (
                self.angle_max
                - self.angle_min
            )
            - 1.0
        )

        return self.clamp(
            normalized,
            -1.0,
            1.0,
        )

    # ================================================================
    # Angle Callback
    # ================================================================

    def angle_callback(
        self,
        msg: Float32,
    ):

        # NaN / Inf 방지
        if not math.isfinite(
            msg.data
        ):

            self.get_logger().warn(
                'Invalid servo angle command'
            )

            return

        # -90 ~ 0 제한
        self.angle_deg = self.clamp(
            msg.data,
            self.angle_min,
            self.angle_max,
        )

        # Degree -> -1 ~ +1
        self.normalized_control = (
            self.angle_to_normalized(
                self.angle_deg
            )
        )

        self.angle_command_received = True

        self.get_logger().info(
            f'Angle: '
            f'{self.angle_deg:.2f} deg '
            f'-> Control: '
            f'{self.normalized_control:.3f}'
        )

    # ================================================================
    # Mode Publish
    # ================================================================

    def publish_mode(self):

        msg = String()

        msg.data = (
            self.current_mode
        )

        self.mode_pub.publish(
            msg
        )

    # ================================================================
    # Servo Control Publish
    # ================================================================

    def publish_control(self):

        # 각도 명령을 한 번도 받지 않았다면
        # 잘못된 기본 Servo 값을 보내지 않음
        if not self.angle_command_received:

            return

        msg = Float32()

        msg.data = float(
            self.normalized_control
        )

        self.control_pub.publish(
            msg
        )

    # ================================================================
    # Timer
    # ================================================================

    def timer_callback(self):

        # ------------------------------------------------------------
        # Parameter 확인
        #
        # 실행 중 ros2 param set도 가능
        # ------------------------------------------------------------

        self.update_mode()

        # ------------------------------------------------------------
        # Mode 계속 Publish
        # ------------------------------------------------------------

        self.publish_mode()

        # ------------------------------------------------------------
        # Servo Control Publish
        #
        # R2 모드에서만 필요한 값이지만
        # 마지막 알고리즘 명령은 계속 전달
        #
        # 실제 선택은 servo_bridge_node가 담당
        # ------------------------------------------------------------

        self.publish_control()

        # ------------------------------------------------------------
        # Log
        # ------------------------------------------------------------

        if self.log_counter % 20 == 0:

            self.get_logger().info(
                f'mode={self.current_mode}, '
                f'angle_received='
                f'{self.angle_command_received}, '
                f'angle='
                f'{self.angle_deg:.2f}, '
                f'control='
                f'{self.normalized_control:.3f}'
            )

        self.log_counter += 1


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ServoControlNode()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        node.get_logger().info(
            'Servo Control Node stopped'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
