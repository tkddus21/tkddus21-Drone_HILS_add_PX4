#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import time


class OffboardControl(Node):
    """PX4 Offboard 모드에서 목표 X, Y, Z 좌표까지 이동 및 자동 착륙"""

    def __init__(self, target_x, target_y, target_z) -> None:
        super().__init__('offboard_control_navigation')

        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher 생성
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscriber 생성 (드론 현재 위치 데이터 수신)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # ✅ vehicle_status 초기화
        self.vehicle_status = VehicleStatus()

        # 목표 위치 설정
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = -abs(target_z)  # PX4 좌표계에서 Z는 음수

        # 현재 위치 저장 변수
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # 도착 판정 변수
        self.arrival_threshold = 0.5  # 목표 지점 ±0.5m 내 도달 여부
        self.arrival_time = None
        self.required_stable_time = 5.0  # 5초 이상 유지 필요
        self.landing_executed = False  # 착륙 실행 여부

        # 타이머 생성 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 오프보드 모드 설정 카운터
        self.offboard_setpoint_counter = 0

    def vehicle_local_position_callback(self, msg):
        """현재 드론 위치 업데이트"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def vehicle_status_callback(self, msg):
        """비행 상태 업데이트"""
        self.vehicle_status = msg

    def publish_vehicle_command(self, command, **params) -> None:
        """PX4에 특정 명령 전송"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """PX4에 ARM 명령 전송"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('✅ ARM 명령 전송 완료')

    def engage_offboard_mode(self):
        """PX4를 Offboard 모드로 전환"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("✅ Offboard 모드 활성화")

    def publish_offboard_control_heartbeat_signal(self):
        """Offboard 모드 활성화 신호 지속 전송"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self):
        """PX4에 목표 위치(X, Y, Z) 전송"""
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def check_arrival_and_land(self):
        """목표 도착 후 착륙 실행"""
        if self.landing_executed:
            return  # 착륙이 이미 실행되었으면 더 이상 수행하지 않음

        if (abs(self.current_x - self.target_x) < self.arrival_threshold and
            abs(self.current_y - self.target_y) < self.arrival_threshold and
            abs(self.current_z - self.target_z) < self.arrival_threshold):

            if self.arrival_time is None:
                self.arrival_time = time.time()
            elif time.time() - self.arrival_time >= self.required_stable_time:
                self.get_logger().info("🎯 목표 지점 도착! 착륙 시작")
                self.land()
        else:
            self.arrival_time = None  # 목표 범위를 벗어나면 초기화

    def land(self):
        """착륙 후 Disarm"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("🛬 착륙 중...")
        self.landing_executed = True  # 중복 실행 방지

        time.sleep(10)  # 착륙 대기
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("⏹️ Disarm 완료")

    def timer_callback(self) -> None:
        """주기적 제어 실행"""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint()
            self.check_arrival_and_land()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    x = float(input("목표 X 좌표: "))
    y = float(input("목표 Y 좌표: "))
    z = float(input("목표 Z 좌표 (양수, m): "))

    rclpy.init(args=args)
    offboard_control = OffboardControl(x, y, z)
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    