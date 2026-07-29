import can
import time
import struct
import threading
import queue
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

# 설정 변수 및 계산 함수
NODE_ID = 5
ENCODER_CPR = 10000
MM_PER_REV = 10.0
EMCY_ID = 0x80 + NODE_ID

# SDO 폴링 주기
SDO_POLLING_INTERVAL_SEC = 0.02

# CANopen 통신 변수
SDO_TX_ID = 0x600 + NODE_ID
NMT_ID = 0x000

# Limit 탐색 설정
SLOW_SEARCH_RPM = 300.0
SEARCH_ACCEL_RPS = 100.0
LIMIT_REVERSE_DISTANCE_MM = 10.0
LIMIT_REVERSE_VELOCITY_RPM = 50.0
LIMIT_REVERSE_MAX_RETRIES = 5
BOTTOM_LIMIT_EMCY_CODE = 0x8311


# 계산 헬퍼 함수
def calculate_accel_data(rpm_per_sec, cpr):
    scaling_factor = cpr / 60.0
    value = int(rpm_per_sec * scaling_factor)
    return list(struct.pack('<I', value))

def calculate_velocity_data(rpm, cpr):
    counts_per_second = int((rpm * cpr) / 60)
    return list(struct.pack('<i', counts_per_second))

def mm_to_encoder_counts(mm, mm_per_rev, cpr):
    counts = (mm / mm_per_rev) * cpr
    return list(struct.pack('<i', int(counts)))

# ----------------------------------------------------


class LiftController(Node):

    def __init__(self):
        super().__init__('lift_controller_node')

        self.declare_parameter('can_interface', 'can2')
        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value

        self.bus = None
        self.running = False
        self.error_queue = queue.Queue()
        self.realtime_position_counts_queue = queue.Queue()
        self.can_thread = None
        self.position_reader_thread = None

        self.reference_position_counts = 0
        self.bottom_limit_mm = None
        self.last_known_position_counts = 0
        self.limit_searching = False
        self.limit_retreating = False
        self.limit_retreat_target_mm = None
        self.limit_retreat_attempts = 0
        self.bottom_limit_found = False
        self.is_stopping = False
        self.initialize_can = False
        # 이동 명령 상태
        self.active_move = False
        self.target_absolute_mm = None
        self.move_done_sent = False
        self.move_tolerance_mm = 1.0   # 도착 판정 허용오차(mm) 필요하면 조절
        # 타이머 변수
        self.position_timer = None
        self.error_timer = None

        # 기본 속도/가속도 값
        # self.default_velocity_rpm = 2000.0
        # self.default_accel_rps = 500.0
        self.default_velocity_rpm = 2000.0
        self.default_accel_rps = 700.0
        self.slow_vel_rpm = 500.0
        self.slow_acc_rps = 500.0
        self.limit_found_publisher = None
        self.current_height_publisher = None
        self.move_done_publisher = None
        self.create_publishers()
        self.create_subscribers()

    def create_publishers(self):
        self.limit_found_publisher = self.create_publisher(Bool, 'lift_status/bottom_limit_found', 10)
        self.get_logger().info('ROS 2 Publisher: lift_status/bottom_limit_found (Bool) 생성 완료.')

        self.current_height_publisher = self.create_publisher(Float64, 'lift_status/current_height', 10)
        self.get_logger().info('ROS 2 Publisher: lift_status/current_height (Float64) 생성 완료.')

        # 🚀 추가: 이동 완료 토픽 (True를 1번만 발행)
        self.move_done_publisher = self.create_publisher(Bool, 'lift_status/move_done', 10)
        self.get_logger().info('ROS 2 Publisher: lift_status/move_done (Bool) 생성 완료.')

    def create_subscribers(self):
        self.find_limit_sub = self.create_subscription(Bool, 'lift_control/find_bottom_limit', self.ros_find_bottom_limit_callback, 10)
        self.get_logger().info('ROS 2 Subscriber: lift_control/find_bottom_limit (Bool) 생성 완료.')

        self.stop_sub = self.create_subscription(Bool, 'lift_control/stop', self.ros_stop_callback, 10)
        self.get_logger().info('ROS 2 Subscriber: lift_control/stop (Bool) 생성 완료.')

        self.move_height_sub = self.create_subscription(Float64, 'lift_control/move_height', self.ros_move_height_callback, 10)
        self.get_logger().info('ROS 2 Subscriber: lift_control/move_height (Float64) 생성 완료.')

        self.slow_move_sub = self.create_subscription(Float64, '/lift_control/slow_move_height', self.ros_move_slow_callback, 10)
        self.get_logger().info('ROS 2 Subscriber: /lift_control/slow_move_height (Float64) 생성 완료.')

    # Bottom Limit 찾기 성공 상태 Publish
    def publish_limit_found_status(self, ok: bool):
        if self.limit_found_publisher:
            msg = Bool()
            msg.data = bool(ok)
            self.limit_found_publisher.publish(msg)
            self.get_logger().info(f'Publishing Bottom Limit Found: {msg.data}')

    # Bottom Limit 기준 현재 높이 Publish 로직
    def publish_current_height(self, current_absolute_mm: float):
        if self.current_height_publisher and self.bottom_limit_mm is not None:
            height_mm = self.bottom_limit_mm - current_absolute_mm
            msg = Float64()
            msg.data = float(height_mm)
            self.current_height_publisher.publish(msg)

    def publish_move_done_once(self):
        if self.move_done_publisher and (not self.move_done_sent):
            msg = Bool()
            msg.data = True
            self.move_done_publisher.publish(msg)
            self.move_done_sent = True
            self.get_logger().info("Publishing Move Done: True (one-shot)")
    # CAN 통신 Helper Functions (self.bus 사용)
    def send_can_message(self, arbitration_id, data):
        if not self.bus: return
        data_padded = data + [0x00] * (8 - len(data))
        message = can.Message(
            arbitration_id=arbitration_id,
            data=data_padded,
            is_extended_id=False
        )
        try:
            self.bus.send(message)
        except can.exceptions.CanOperationError as e:
            pass

    def send_sdo_write(self, index, subindex, data, data_len):
        csid = {1: 0x2F, 0: 0x2F, 2: 0x2B, 4: 0x23}.get(data_len)
        index_low = index & 0xFF
        index_high = (index >> 8) & 0xFF

        sdo_data = data
        payload = [csid, index_low, index_high, subindex] + sdo_data
        self.send_can_message(SDO_TX_ID, payload)

    def read_current_position_sdo(self):
        if not self.bus: return None

        request_data = [0x40, 0x64, 0x60, 0x00]
        self.send_can_message(SDO_TX_ID, request_data)

        response_id = 0x580 + NODE_ID
        start_time = time.time()

        while time.time() - start_time < 0.5:
            try:
                msg = self.bus.recv(timeout=0.01)
                if msg and msg.arbitration_id == response_id and msg.data[0] == 0x43:
                    position_counts = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                    position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
                    return position_mm
                elif msg and msg.arbitration_id == response_id and msg.data[0] == 0x80:
                    self.get_logger().error("SDO Read Abort during position capture.")
                    return None
            except:
                return None
        return None

    def log_message(self, level, message):
        """메시지를 ROS 로그에 기록합니다."""
        if level == 'ERROR':
            self.get_logger().error(message)
        elif level == 'WARNING':
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def clear_pending_emcy_errors(self):
        while True:
            try:
                self.error_queue.get_nowait()
            except queue.Empty:
                break

    def recover_motor_to_position_mode(self, target_counts, velocity_rpm=None, accel_rps=None):
        target_data = list(struct.pack('<i', int(target_counts)))

        self.send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2); time.sleep(0.1)
        self.send_sdo_write(0x60FF, 0x00, [0x00, 0x00, 0x00, 0x00], 4); time.sleep(0.05)
        self.send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)

        if velocity_rpm is not None:
            velocity_data = calculate_velocity_data(velocity_rpm, ENCODER_CPR)
            self.send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)

        if accel_rps is not None:
            accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)
            self.send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)

        self.send_sdo_write(0x607A, 0x00, target_data, 4); time.sleep(0.05)
        self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2); time.sleep(0.1)
        self.send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2); time.sleep(0.1)
        self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)

    def command_bottom_limit_retreat(self):
        if self.bottom_limit_mm is None:
            self.limit_retreating = False
            self.publish_limit_found_status(False)
            self.log_message('ERROR', "Bottom Limit 후퇴 불가: 저장된 Bottom Limit 위치가 없습니다.")
            return False

        if self.limit_retreat_attempts >= LIMIT_REVERSE_MAX_RETRIES:
            self.limit_retreating = False
            self.bottom_limit_found = False
            self.publish_limit_found_status(False)
            self.log_message(
                'ERROR',
                f"Bottom Limit 후퇴 실패: {LIMIT_REVERSE_MAX_RETRIES}회 재시도 후에도 0x{BOTTOM_LIMIT_EMCY_CODE:04X} 오류가 계속 발생했습니다."
            )
            return False

        self.limit_retreat_attempts += 1
        self.limit_retreating = True
        self.limit_retreat_target_mm = self.bottom_limit_mm - LIMIT_REVERSE_DISTANCE_MM
        target_counts = int((self.limit_retreat_target_mm / MM_PER_REV) * ENCODER_CPR)

        try:
            self.clear_pending_emcy_errors()
            self.recover_motor_to_position_mode(
                target_counts,
                velocity_rpm=LIMIT_REVERSE_VELOCITY_RPM,
                accel_rps=SEARCH_ACCEL_RPS,
            )

            self.send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2); time.sleep(0.1)
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)

            position_mm = (self.last_known_position_counts / ENCODER_CPR) * MM_PER_REV
            self.log_message(
                'INFO',
                f"Bottom Limit 후퇴 시작({self.limit_retreat_attempts}/{LIMIT_REVERSE_MAX_RETRIES}): "
                f"{position_mm:.4f} mm에서 {LIMIT_REVERSE_DISTANCE_MM} mm 떨어진 "
                f"{self.limit_retreat_target_mm:.4f} mm로 이동."
            )
            return True

        except Exception as e:
            self.limit_retreating = False
            self.bottom_limit_found = False
            self.publish_limit_found_status(False)
            self.get_logger().error(f"Bottom Limit 후퇴 명령 중 예외 발생: {e}")
            return False


    # 비동기 CAN 리스너 및 위치 업데이트
    def position_reader_thread_func(self):
        read_position_request = can.Message(
            arbitration_id=SDO_TX_ID,
            data=[0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00],
            is_extended_id=False
        )

        while self.running:
            if self.bus:
                try:
                    self.bus.send(read_position_request)
                except Exception as e:
                    pass
            time.sleep(SDO_POLLING_INTERVAL_SEC)

    def can_listener_thread(self):
        sdo_response_id = 0x580 + NODE_ID

        while self.running:
            try:
                msg = self.bus.recv(timeout=0.01)

                if msg and msg.arbitration_id == EMCY_ID:
                    error_code = struct.unpack('<H', bytes(msg.data[:2]))[0]
                    if error_code != 0x0000:
                        error_register = msg.data[2]
                        self.error_queue.put((error_code, error_register, msg.data))

                elif msg and msg.arbitration_id == sdo_response_id and msg.data[0] == 0x43:
                    position_counts = struct.unpack('<i', bytes(msg.data[4:8]))[0]
                    self.realtime_position_counts_queue.put(position_counts)

            except AttributeError:
                break
            except Exception as e:
                self.get_logger().error(f"리스너 스레드 오류: {e}")
                break

    # Bottom Limit 기준 현재 높이 업데이트 및 Publish 로직
    def update_realtime_position(self):
        if self.running and not self.realtime_position_counts_queue.empty():
            position_counts = None
            while not self.realtime_position_counts_queue.empty():
                position_counts = self.realtime_position_counts_queue.get()

            if position_counts is not None:
                self.last_known_position_counts = position_counts
                position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
                if self.bottom_limit_mm is not None:
                    self.publish_current_height(position_mm)

                # 이동 종료 판정 (목표 근처 도달하면 move_done True 1회)
                if self.active_move and (self.target_absolute_mm is not None):
                    if abs(position_mm - self.target_absolute_mm) <= self.move_tolerance_mm:
                        self.active_move = False
                        self.publish_move_done_once()

                if self.limit_retreating and (self.limit_retreat_target_mm is not None):
                    if abs(position_mm - self.limit_retreat_target_mm) <= self.move_tolerance_mm:
                        self.limit_retreating = False
                        self.limit_retreat_attempts = 0
                        self.bottom_limit_found = True
                        self.publish_limit_found_status(True)
                        self.log_message(
                            'SUCCESS',
                            f"Bottom Limit 후퇴 완료: 목표 {self.limit_retreat_target_mm:.4f} mm, "
                            f"현재 {position_mm:.4f} mm."
                        )

        if self.running:
            self.position_timer = threading.Timer(SDO_POLLING_INTERVAL_SEC, self.update_realtime_position)
            self.position_timer.start()

    # Bottom Limit 발견 시 상태 Topic 발행 로직
    def check_for_errors(self):

        if not self.error_queue.empty():
            error_code, error_register, _ = self.error_queue.get()

            if self.active_move and not self.limit_retreating:
                self.active_move = False
                self.publish_move_done_once()
            error_message = (
                f"CANopen Emergency Message (EMCY) 수신!\n"
                f"오류 코드: 0x{error_code:04X}, 레지스터: 0x{error_register:02X}"
            )

            try:
                if self.limit_searching:
                    self.send_sdo_write(0x60FF, 0x00, [0x00, 0x00, 0x00, 0x00], 4); time.sleep(0.05)
                    try: self.send_sdo_write(0x6040, 0x00, [0x02, 0x00], 2); time.sleep(0.1)
                    except: pass
                    try: self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2); time.sleep(0.1)
                    except: pass

                    time.sleep(1.0)

                    position_counts = self.last_known_position_counts
                    position_mm = (position_counts / ENCODER_CPR) * MM_PER_REV
                    self.bottom_limit_mm = position_mm
                    self.limit_searching = False
                    self.limit_retreating = False
                    self.limit_retreat_attempts = 0
                    self.bottom_limit_found = False
                    self.log_message('SUCCESS', f"Bottom Limit 저장: {position_mm:.4f} mm (안정화 후 검출 시점)")
                    self.log_message('WARNING', f"Bottom Limit 감지: {error_message}\n후퇴 명령을 시작합니다.")
                    self.command_bottom_limit_retreat()

                elif self.limit_retreating and error_code == BOTTOM_LIMIT_EMCY_CODE:
                    self.log_message(
                        'WARNING',
                        f"Bottom Limit 후퇴 중 0x{BOTTOM_LIMIT_EMCY_CODE:04X} 재발. "
                        "모터 오류를 복구하고 후퇴 명령을 다시 보냅니다."
                    )
                    self.command_bottom_limit_retreat()

                else:
                    self.log_message('ERROR', f"🚨 모터 오류 발생: {error_message}")

                    if self.limit_retreating:
                        self.limit_retreating = False
                        self.bottom_limit_found = False
                        self.publish_limit_found_status(False)

                    self.recover_motor_to_position_mode(self.last_known_position_counts)
                    self.get_logger().info("모터 EMCY 오류 초기화 및 Position Mode 상태로 복구 완료.")

            except Exception as e:
                self.get_logger().error(f"오류 복구/후퇴 중 예외 발생: {e}")
                self.limit_searching = False
                self.limit_retreating = False
                pass

        if self.running:
            self.error_timer = threading.Timer(SDO_POLLING_INTERVAL_SEC, self.check_for_errors)
            self.error_timer.start()


    # 컨트롤러 함수 (메서드)

    def initialize_can_bus(self):
        if self.initialize_can:
            return
        try:
            if self.error_timer:
                self.error_timer.cancel()
            if self.position_timer:
                self.position_timer.cancel()

            if self.bus: self.bus.shutdown()

            self.bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

            if self.can_thread is None or not self.can_thread.is_alive():
                self.can_thread = threading.Thread(target=self.can_listener_thread, daemon=True)
                self.can_thread.start()

            if self.position_reader_thread is None or not self.position_reader_thread.is_alive():
                self.position_reader_thread = threading.Thread(target=self.position_reader_thread_func, daemon=True)
                self.position_reader_thread.start()
            # self.can_thread = threading.Thread(target=self.can_listener_thread, daemon=True)
            # self.can_thread.start()

            # self.position_reader_thread = threading.Thread(target=self.position_reader_thread_func, daemon=True)
            # self.position_reader_thread.start()

            # 타이머 시작
            self.error_timer = threading.Timer(SDO_POLLING_INTERVAL_SEC, self.check_for_errors)
            self.error_timer.start()

            self.position_timer = threading.Timer(SDO_POLLING_INTERVAL_SEC, self.update_realtime_position)
            self.position_timer.start()

            self.send_can_message(NMT_ID, [0x81, 0x00]); time.sleep(2.0)
            self.send_sdo_write(0x1017, 0x00, [0xE8, 0x03], 2); time.sleep(0.2)
            self.send_can_message(NMT_ID, [0x01, NODE_ID]); time.sleep(1.0)

            self.send_sdo_write(0x6040, 0x00, [0x80, 0x00], 2)
            self.send_sdo_write(0x6040, 0x00, [0x00, 0x00], 2)

            zero_data = [0x00, 0x00, 0x00, 0x00]
            self.send_sdo_write(0x6064, 0x00, zero_data, 4)
            time.sleep(0.1)

            self.send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.1)
            self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)

            self.log_message('SUCCESS', "CAN 버스 연결 및 모터 준비 완료. 자동 초기화 성공!")
            self.initialize_can = True

        except Exception as e:
            self.log_message('ERROR', f"CAN 버스 초기화 실패: {e}")
            self.initialize_can = False
            # 1초 후 재시도 타이머 설정
            threading.Timer(1.0, self.initialize_can_bus).start()
            # self.running = False


    def set_reference_position(self):
        position_mm = self.read_current_position_sdo()
        if position_mm is not None:
            self.reference_position_counts = int((position_mm / MM_PER_REV) * ENCODER_CPR)
            self.log_message('INFO', f"기준 위치 설정 완료: 현재 위치 {position_mm:.4f} mm로 설정했습니다.")
        else:
            self.log_message('ERROR', "현재 위치를 읽는 데 실패했습니다.")

    # Absolute Position Mode 구동 함수 (Topic에서 사용)
    def run_absolute_position(self, target_absolute_mm, velocity_rpm=None, accel_rps=None):
        if not self.bus:
            self.get_logger().error("CAN 버스가 연결되지 않았습니다.")
            self.log_message('WARNING', "CAN 버스가 연결되지 않았습니다.")
            return
        self.position_stop_motor(); time.sleep(0.5)
        # 기본값 사용
        if velocity_rpm is None:
            velocity_rpm = self.default_velocity_rpm
        if accel_rps is None:
            accel_rps = self.default_accel_rps

        try:
            # self.send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.05)
            # self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
            # self.send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2)
            # self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
            # time.sleep(0.5)
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)
            self.send_sdo_write(0x6060, 0x00, [0x01], 1); time.sleep(0.1)


            position_data = mm_to_encoder_counts(target_absolute_mm, MM_PER_REV, ENCODER_CPR)
            velocity_data = calculate_velocity_data(velocity_rpm, ENCODER_CPR)
            accel_data = calculate_accel_data(accel_rps, ENCODER_CPR)

            self.send_sdo_write(0x6081, 0x00, velocity_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x607A, 0x00, position_data, 4); time.sleep(0.05)

            # sleep을 줘야 간헐적으로 이동명령 거부되는 것을 막음
            self.send_sdo_write(0x6040, 0x00, [0x1F, 0x00], 2); time.sleep(0.1)
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)

            self.log_message('INFO', f"절대 위치 모드 시작: 목표 위치 {target_absolute_mm} mm로 이동 (속도: {velocity_rpm} RPM)")

        except Exception as e:
            self.log_message('ERROR', f"위치 모드 명령 중 오류 발생: {e}")


    def position_stop_motor(self):
        if not self.bus or self.is_stopping:
            self.log_message('WARNING', "CAN 버스가 연결되지 않았습니다. 정지 명령을 무시합니다.")
            return
        try:
            self.is_stopping = True
            # 감속값을 일시적으로 높게 설정
            fast_decel = calculate_accel_data(7000.0, ENCODER_CPR)
            self.send_sdo_write(0x6084, 0x00, fast_decel, 4)
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x01], 2) #halts

            threading.Timer(0.3, self.revert_stop_process).start()
            self.log_message('INFO', "급정지 실행, 0.3초 후 복구 예정.")
        except Exception as e:
            self.is_stopping = False # 에러 시 잠금 해제
            self.log_message('ERROR', f"정지 명령 중 오류 발생: {e}")

    def revert_stop_process(self):
        try:
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2)
            original_decel = calculate_accel_data(self.default_accel_rps, ENCODER_CPR)
            self.send_sdo_write(0x6084, 0x00, original_decel, 4)
            self.is_stopping = False
            self.log_message('INFO', "급정지 복구 완료. 이동 가능.")
        except Exception as e:
            self.is_stopping = False
            self.log_message('ERROR', f"복구 중 오류: {e}")

    # Bottom Limit 찾기 함수
    def find_bottom_limit(self):
        if not self.bus:
            self.publish_limit_found_status(False)
            return self.log_message('WARNING', "CAN 버스가 연결되지 않았습니다.")
        if not self.initialize_can:
            self.publish_limit_found_status(False)
            return self.log_message('WARNING', "CAN 버스 초기화가 완료되지 않았습니다.")

        if self.limit_searching or self.limit_retreating:
            self.publish_limit_found_status(False)
            return self.log_message('WARNING', "이미 Bottom Limit 캘리브레이션 중입니다.")

        try:
            self.limit_searching = True
            self.limit_retreating = False
            self.limit_retreat_target_mm = None
            self.limit_retreat_attempts = 0
            self.bottom_limit_found = False
            self.clear_pending_emcy_errors()

            # Bottom 방향(+)으로 이동
            target_rpm = SLOW_SEARCH_RPM
            self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2); time.sleep(0.1)
            self.send_sdo_write(0x6060, 0x00, [0x03], 1); time.sleep(0.05)
            self.send_sdo_write(0x6040, 0x00, [0x07, 0x00], 2); time.sleep(0.05)
            self.send_sdo_write(0x6040, 0x00, [0x0F, 0x00], 2); time.sleep(0.1)

            velocity_data = calculate_velocity_data(target_rpm, ENCODER_CPR)
            accel_data = calculate_accel_data(SEARCH_ACCEL_RPS, ENCODER_CPR)

            self.send_sdo_write(0x6083, 0x00, accel_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x6084, 0x00, accel_data, 4); time.sleep(0.05)
            self.send_sdo_write(0x60FF, 0x00, velocity_data, 4)

            self.log_message('INFO', f"Bottom Limit 찾기 시작: 모터가 + 방향으로 {SLOW_SEARCH_RPM} RPM 구동.")

        except Exception as e:
            self.log_message('ERROR', f"Bottom Limit 찾기 명령 중 오류 발생: {e}")
            self.limit_searching = False
            self.publish_limit_found_status(False)

    # Bottom Limit 찾기 명령 Topic 콜백 함수
    def ros_find_bottom_limit_callback(self, msg: Bool):
        if not msg.data:
            return
        self.log_message('INFO', 'ROS Topic 수신: Bottom Limit 찾기(True)')
        self.find_bottom_limit()

    def ros_stop_callback(self, msg: Bool):
        if not msg.data:
            return
        self.log_message('INFO', 'ROS Topic 수신: Stop(True)')

        calibration_active = self.limit_searching or self.limit_retreating
        self.limit_searching = False
        self.limit_retreating = False
        self.limit_retreat_target_mm = None
        self.limit_retreat_attempts = 0

        # 이동 중이었다면 "이동 종료"를 True 1번 발행
        if self.active_move:
            self.active_move = False
            self.target_absolute_mm = None
            self.publish_move_done_once()

        if calibration_active:
            self.bottom_limit_found = False
            self.publish_limit_found_status(False)
            self.log_message('WARNING', 'Bottom Limit 캘리브레이션을 중지했습니다.')

        self.position_stop_motor()

    def ros_move_height_callback(self, msg: Float64):
        if self.bottom_limit_mm is None:
            self.log_message('ERROR', "ROS 높이 이동 불가: Bottom Limit 미설정.")
            return

        if self.limit_searching or self.limit_retreating:
            self.log_message('WARNING', "Bottom Limit 캘리브레이션 중입니다. 높이 이동 명령을 무시합니다.")
            return

        if self.is_stopping:
            self.log_message('WARNING', "모터 정지 중입니다. 이동 명령을 무시합니다.")
            return

        # 이동 중이면 새 이동 명령 무시
        # if self.active_move:
        #     self.log_message('WARNING', "이동 중입니다. 새 이동 명령을 무시합니다.")
        #     return

        target_height_mm = float(msg.data)
        target_absolute_mm = self.bottom_limit_mm - target_height_mm

        self.log_message('INFO', f'ROS Topic 수신: 높이 {target_height_mm:.4f} mm (목표 절대: {target_absolute_mm:.4f} mm)')

        # 이동 상태 설정
        self.target_absolute_mm = target_absolute_mm
        self.active_move = True
        self.move_done_sent = False

        self.run_absolute_position(target_absolute_mm)

    def ros_move_slow_callback(self, msg: Float64):
        # if self.bottom_limit_mm is None or self.is_stopping or self.active_move:
        if self.bottom_limit_mm is None or self.is_stopping or self.limit_searching or self.limit_retreating:
            return

        target_height_mm = float(msg.data)
        target_absolute_mm = self.bottom_limit_mm - target_height_mm

        self.target_absolute_mm = target_absolute_mm
        self.active_move = True
        self.move_done_sent = False
        self.run_absolute_position(target_absolute_mm, velocity_rpm=self.slow_vel_rpm, accel_rps=self.slow_acc_rps)

    def cleanup(self):
        """종료 시 정리 작업"""
        self.log_message('INFO', "노드 종료 및 ROS/CAN 통신 종료 요청.")
        self.running = False

        # 타이머 취소
        if self.position_timer:
            self.position_timer.cancel()
        if self.error_timer:
            self.error_timer.cancel()

        # 스레드 종료 대기
        if self.can_thread and self.can_thread.is_alive():
            self.can_thread.join(timeout=0.2)
        if self.position_reader_thread and self.position_reader_thread.is_alive():
            self.position_reader_thread.join(timeout=0.2)

        # CAN 버스 종료
        if self.bus:
            try:
                self.send_sdo_write(0x6040, 0x00, [0x06, 0x00], 2)
                self.bus.shutdown()
            except:
                pass


# --- Main 실행 블록 ---

def main(args=None):
    rclpy.init(args=args)

    node = LiftController()
    node.running = True

    node.get_logger().info("Lift Controller 시작 (Headless Mode)")

    # CAN 초기화 (0.1초 후 시작)
    init_timer = threading.Timer(0.1, node.initialize_can_bus)
    init_timer.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt 수신, 종료 중...")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
