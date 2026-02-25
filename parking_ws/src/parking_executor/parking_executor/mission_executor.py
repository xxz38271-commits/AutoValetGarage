#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from parking_msgs.msg import MissionArray, MissionStatus
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped

# 🔔 삐뽀 사운드용
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

# TurtleBot4 dock/undock - 직접 액션 사용
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

# 라인 정렬용
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class BeepNode:
    def __init__(self, node, namespace: str):
        self.node = node
        self.pub = node.create_publisher(
            AudioNoteVector,
            f"/{namespace}/cmd_audio",
            10
        )

    def play_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(nanosec=300_000_000)),
        ]
        self.node.get_logger().info("[삐뽀] 재생")
        self.pub.publish(msg)


class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')

        # 현재 진행 중인 미션 / 큐
        self.current_mission = None
        self.mission_queue = []

        # 네임스페이스(/robot1, /robot5)에서 로봇 이름 추출
        self.robot_name = self.get_namespace().replace('/', '')
        # /robotX/initialpose 퍼블리셔 (amcl 초기자세 설정용)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )

        self.get_logger().info(f"MissionExecutor started for robot: {self.robot_name}")
                # 🔔 삐뽀 노드
        self.beep = BeepNode(self, self.robot_name)
        # 🔹 미션이 전혀 없는 상태(idle) 타이머 (자동 도킹용)
        self.idle_start_time = None
        self.idle_dock_triggered = False


        # 🔹 도킹/언도킹 액션 클라이언트 (TurtleBot4Navigator 대신)
        self.dock_client = ActionClient(self, Dock, f'/{self.robot_name}/dock')
        self.undock_client = ActionClient(self, Undock, f'/{self.robot_name}/undock')
        self.get_logger().info("Dock/Undock action clients created.")
        
        # 🔹 도킹 상태를 실시간으로 확인하는 subscriber
        self.is_docked = None  # 초기값은 None (아직 모름)
        self.dock_status_received = False
        
        # 🔹 비동기 언도킹/도킹 상태 관리
        self.undocking_in_progress = False
        self.undock_goal_handle = None
        self.undock_result_future = None
        
        self.docking_in_progress = False
        self.dock_goal_handle = None
        self.dock_result_future = None
        
        self.dock_status_sub = self.create_subscription(
            DockStatus,
            'dock_status',
            self.dock_status_callback,
            10
        )
        
        # 🔹 초기 도킹 상태 기본값 설정 (토픽 수신 후 자동 업데이트됨)
        # robot1과 robot5 모두 도킹된 상태로 시작
        self.is_docked = True
        self.get_logger().info(
            f"📍 {self.robot_name} 초기 도킹 상태 (기본값): DOCKED 🔌 "
            f"(실제 상태는 토픽 수신 후 자동 업데이트)"
        )
        
        # 🔹 robot5만 initial pose 설정
        if self.robot_name == "robot5":
            self.get_logger().info("📍 robot5 도킹 상태 → initial pose 설정")
            self.publish_robot5_initial_pose()

        # 출차/이동 미션 구독 (TaskAllocator에서 나오는 토픽)
        self.mission_sub = self.create_subscription(
            MissionArray,
            'assigned_missions',
            self.mission_callback,
            10
        )

        # 미션 상태 퍼블리시
        self.status_pub = self.create_publisher(
            MissionStatus,
            'mission_status',
            10
        )

        # Nav2 NavigateToPose 액션 클라이언트
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_name}/navigate_to_pose'
        )

        # 🔹 라인 정렬용 카메라/속도 토픽
        #   - 이미지 토픽 이름은 환경에 맞게 필요하면 launch에서 remap 해도 됨
        self.image_sub = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',   # 예: /robot1/camera/image/compressed 로 remap 가능
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.bridge = CvBridge()

        # 🔹 라인 정렬 상태 변수
        self.enable_line_detect = False   # ALIGN 미션일 때만 True
        self.align_active = False         # ALIGN 미션 진행 중인지
        self.phase = "IDLE"               # ALIGN_MOVE / FINAL_FORWARD / DONE

        self.callback_count = 0
        self.last_callback_time = None

        self.scan_start_time = None
        self.phase3_start_time = None
        self.stable_count = 0
        self.stable_needed = 8

        # 제어 파라미터 (필요하면 숫자 조절)
        self.kp_ang = 0.003
        self.max_angular_z = 0.5
        self.align_linear_speed = 0.05
        self.dead_band_px = 5.0
        self.center_tolerance_px = 10.0

        self.final_forward_speed = 0.05
        self.final_forward_duration = 0  # 정렬 후 직진하는 시간(s)
        # 🔹 라인 미검출/이미지 실패 시간 추적용
        self.no_line_start_time = None

        # Nav2 상태
        self.nav_goal_handle = None
        self.nav_result_future = None

        # 주기적으로 Nav2 미션 처리
        self.timer = self.create_timer(0.5, self.process_missions)

        self.get_logger().info('MissionExecutor fully initialized (Nav2 + Dock + Line Align).')
    
    # ---------------------------------------------------------
    # 도킹 상태 콜백 (실시간 업데이트)
    # ---------------------------------------------------------
    def dock_status_callback(self, msg: DockStatus):
        """도킹 상태를 실시간으로 업데이트합니다."""
        self.is_docked = msg.is_docked
        
        if not self.dock_status_received:
            # 첫 메시지일 때만 로깅
            self.dock_status_received = True
            self.get_logger().info(
                f"✅ 도킹 상태 수신: {'DOCKED' if msg.is_docked else 'UNDOCKED'}"
            )

    # ---------------------------------------------------------
    # 미션 수신 콜백
    # ---------------------------------------------------------
    def mission_callback(self, msg: MissionArray):
        for m in msg.missions:
            self.mission_queue.append(m)
            self.get_logger().info(f'Received mission_id={m.mission_id}, type={m.mission_type}')

    # ---------------------------------------------------------
    # 타이머 콜백: 현재 미션 진행 또는 다음 미션 시작
    # ---------------------------------------------------------
    def process_missions(self):
        # 🔹 언도킹 진행 중이면 결과 체크
        if self.undocking_in_progress:
            self.check_undock_result()
            return
            
        # 🔹 도킹 진행 중이면 결과 체크
        if self.docking_in_progress:
            self.check_dock_result()
            return
            
        # 🔹 라인 정렬 진행 중이면 Nav2 새 미션 시작 금지
        if self.align_active:
            # ✅ 카메라 토픽 무응답 2초 타임아웃 처리
            now = time.time()
            # ALIGN 시작은 했는데, 아직 한 번도 이미지 콜백이 안 들어온 경우
            if self.align_start_time is not None and self.last_image_time is None:
                elapsed = now - self.align_start_time
                if elapsed >= 2.0:
                    self.get_logger().warn(
                        f"⏱ {elapsed:.1f}s 동안 카메라 토픽 수신 없음 → ALIGN 미션을 통과 처리하고 다음으로 진행."
                    )
                    self.finish_align_mission(succeeded=True)
            return
        # 🔹 현재 미션이 있거나, 큐에 미션이 하나라도 있으면 idle 아님 → 타이머 리셋
        if self.current_mission is not None or self.mission_queue:
            self.idle_start_time = None
            self.idle_dock_triggered = False
        else:
            # 🔹 완전 idle 상태면, 자동 도킹 체크
            self.handle_idle_dock()
            # 미션이 없으니 여기서 끝
            return

        # 🔹 Nav2 미션이 진행 중이면 결과만 체크
        if self.current_mission is not None:
            self.check_nav_result()
            return

        # 🔹 대기 중인 미션이 있으면 하나 꺼내서 시작
        if self.mission_queue:
            mission = self.mission_queue.pop(0)
            self.current_mission = mission
            self.get_logger().info(f'Start mission_id={mission.mission_id}, type={mission.mission_type}')
            self.send_nav_goal_for_mission(mission)

    # ---------------------------------------------------------
    # Nav2 목표 전송 (+ 출발 전 undock) 또는 라인 정렬 시작
    # ---------------------------------------------------------
    def send_nav_goal_for_mission(self, mission):
        # 🔔 0) BEEP 미션은 네비 없이 삐뽀만 울리고 종료
        if mission.mission_type in ['BEEP_R1', 'BEEP_R5']:
            self.get_logger().info(
                f"BEEP mission {mission.mission_id} for {self.robot_name} → 삐뽀 재생"
            )
            # 삐뽀 재생
            self.beep.play_beep()
            # 미션 상태 SUCCEEDED로 발행
            self.publish_status(
                mission.mission_id,
                'SUCCEEDED',
                1.0,
                'Beep played'
            )
            # 현재 미션 종료하고 다음 미션으로
            self.current_mission = None
            return
        # 0) ALIGN_BASE_* 미션은 Nav2 대신 "카메라 기반 라인 정렬" 모드로 처리
        if mission.mission_type in ['ALIGN_BASE_FRONT', 'ALIGN_BASE_EXIT']:
            self.get_logger().info(
                f'ALIGN mission {mission.mission_id} 시작 → 카메라 라인 정렬 모드 진입'
            )
            # 라인 정렬 상태 초기화
            self.align_active = True
            self.enable_line_detect = True
            self.phase = "ALIGN_MOVE"
            self.callback_count = 0
            self.stable_count = 0
            self.scan_start_time = None
            self.phase3_start_time = None

            # 🔹 타임아웃 기준 초기화
            self.align_start_time = time.time()
            self.last_image_time = None
            self.no_line_start_time = None

            # 상태 퍼블리시
            self.publish_status(
                mission.mission_id,
                'RUNNING',
                0.0,
                'Aligning on lane line'
            )
            return

        # 1) 그 외 미션은 Nav2 + undock
        if self.is_docked:
            self.get_logger().info("📍 Robot is docked. Starting undock...")
            self.start_undock()
            return  # 언도킹 완료 후 다시 이 미션을 처리
        else:
            self.get_logger().info("Robot is already undocked. Proceeding to navigation.")

        # 2) Nav2 액션 서버 준비 여부 확인
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 NavigateToPose server not ready yet, will retry.')
            # 다시 큐에 넣고 다음 타이머 사이클에서 재시도
            self.mission_queue.insert(0, mission)
            self.current_mission = None
            return

        # 3) NavigateToPose Goal 생성
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = mission.target_pose
        goal_msg.pose = goal_pose

        # 상태: RUNNING
        self.publish_status(mission.mission_id, 'RUNNING', 0.0, 'Navigation started')

        # 4) Nav2 비동기 goal 전송
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    # ---------------------------------------------------------
    # Nav2 goal 응답 콜백
    # ---------------------------------------------------------
    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            if self.current_mission:
                self.publish_status(self.current_mission.mission_id, 'FAILED', 0.0, 'Goal rejected')
                self.current_mission = None
            return

        self.get_logger().info('Nav2 goal accepted.')
        self.nav_goal_handle = goal_handle
        self.nav_result_future = goal_handle.get_result_async()

    # ---------------------------------------------------------
    # Nav2 피드백 콜백 (현재는 사용 안 함)
    # ---------------------------------------------------------
    def nav_feedback_callback(self, feedback_msg):
        pass

    # ---------------------------------------------------------
    # Nav 결과 체크
    # ---------------------------------------------------------
    def check_nav_result(self):
        if self.nav_result_future is None:
            return
        if not self.nav_result_future.done():
            return

        result = self.nav_result_future.result()

        if self.current_mission is None:
            return

        if result.status == 4:  # SUCCEEDED
            self.publish_status(self.current_mission.mission_id, 'SUCCEEDED', 1.0, 'Navigation succeeded')
            self.get_logger().info(f'Mission {self.current_mission.mission_id} completed.')
        else:
            self.publish_status(self.current_mission.mission_id, 'FAILED', 0.0, f'Nav2 error status={result.status}')
            self.get_logger().warn(f'Mission {self.current_mission.mission_id} FAILED. Nav2 status={result.status}')

        self.current_mission = None
        self.nav_goal_handle = None
        self.nav_result_future = None

    # ---------------------------------------------------------
    # 미션 상태 퍼블리시
    # ---------------------------------------------------------
    def publish_status(self, mission_id: int, state: str, progress: float, message: str):
        msg = MissionStatus()
        msg.mission_id = mission_id
        msg.state = state
        msg.progress = progress
        msg.message = message
        self.status_pub.publish(msg)

    # ---------------------------------------------------------
    # 카메라 콜백: 라인 검출 + 정렬 제어 (ALIGN_* 미션일 때만 동작)
    # ---------------------------------------------------------
    def image_callback(self, msg: CompressedImage):
        """카메라 이미지 콜백 - 라인 검출 기반 정렬"""
        # ALIGN 모드가 아니면 바로 리턴
        if not self.align_active or not self.enable_line_detect:
            return

        if (self.current_mission is None or
                self.current_mission.mission_type not in ['ALIGN_BASE_FRONT', 'ALIGN_BASE_EXIT']):
            return

        self.callback_count += 1
        self.last_callback_time = self.get_clock().now()

        # 🔹 카메라 이미지가 최소 한 번은 들어왔다는 표시
        now = time.time()
        self.last_image_time = now

        # 정렬 완료 상태면 정지 명령만 보내고 리턴
        cmd = Twist()
        if self.phase == "DONE":
            self.cmd_pub.publish(cmd)
            return

        # 1) CompressedImage -> OpenCV BGR
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 실패: {e}")

            # 🔹 디코딩 실패도 '라인을 못 본 것'으로 간주하고 4초 타임아웃 체크
            now = time.time()
            if self.no_line_start_time is None:
                self.no_line_start_time = now
            else:
                if now - self.no_line_start_time >= 4.0:
                    self.get_logger().warn("⏱ 4초 동안 이미지 디코딩 실패 → ALIGN 미션을 통과 처리하고 다음으로 진행.")
                    self.finish_align_mission(succeeded=True)
            return

        h, w, _ = frame.shape

        # 2) ROI 설정 (아랫쪽 1/3만 사용)
        roi_y_start = int(h * 2 / 3)
        roi = frame[roi_y_start:h, :]

        # 3) 흰 선 검출 (HSV 필터)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([179, 30, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 4) 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        # 5) 컨투어 찾기
        contours, _ = cv2.findContours(
            mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        line_found = False
        lane_center_x = None

        if contours:
            min_area = 300.0
            min_aspect = 2.0

            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area:
                    continue

                x, y, cw, ch = cv2.boundingRect(c)
                aspect = ch / float(cw + 1e-3)
                if aspect < min_aspect:
                    continue

                candidates.append((area, c))

            if candidates:
                image_center_x = w / 2.0
                best_c = None
                min_distance_to_center = float('inf')

                for area, c in candidates:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = M["m10"] / M["m00"]
                        dist = abs(cx - image_center_x)
                        if dist < min_distance_to_center:
                            min_distance_to_center = dist
                            best_c = c

                if best_c is not None:
                    M = cv2.moments(best_c)
                    if M["m00"] > 0:
                        cx_global = (M["m10"] / M["m00"])
                        cy_global = (M["m01"] / M["m00"]) + roi_y_start

                        x, y, cw, ch = cv2.boundingRect(best_c)
                        cv2.rectangle(frame, (x, y + roi_y_start),
                                      (x + cw, y + ch + roi_y_start), (0, 255, 0), 2)
                        cv2.circle(frame, (int(cx_global), int(cy_global)), 5, (0, 0, 255), -1)

                        lane_center_x = cx_global
                        line_found = True

        # ---- 제어 로직 ----
        if line_found and lane_center_x is not None:
            self.no_line_start_time = None
            self.scan_start_time = None

            cv2.line(frame, (int(lane_center_x), roi_y_start),
                     (int(lane_center_x), h), (255, 0, 0), 2)

            image_center_x = w / 2.0
            error_px = lane_center_x - image_center_x

            cv2.putText(frame, f"Phase: {self.phase}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Error: {error_px:.1f}px", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Stable: {self.stable_count}/{self.stable_needed}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            raw_omega = -self.kp_ang * error_px
            if raw_omega > self.max_angular_z:
                raw_omega = self.max_angular_z
            elif raw_omega < -self.max_angular_z:
                raw_omega = -self.max_angular_z

            # ---- Phase: ALIGN_MOVE (정렬 + 천천히 전진)
            if self.phase == "ALIGN_MOVE":
                cmd.linear.x = self.align_linear_speed

                if abs(error_px) < self.dead_band_px:
                    cmd.angular.z = 0.0
                else:
                    cmd.angular.z = float(raw_omega)

                # 중앙 근처에 일정 시간 유지되면 정렬 완료
                if abs(error_px) < self.center_tolerance_px:
                    self.stable_count += 1
                else:
                    self.stable_count = 0

                if self.stable_count >= self.stable_needed:
                    self.phase = "FINAL_FORWARD"
                    self.phase3_start_time = time.time()
                    self.get_logger().info("✅ 정렬 완료! 최종 직진 시작")

            # ---- Phase: FINAL_FORWARD (짧게 직진 후 완료)
            elif self.phase == "FINAL_FORWARD":
                elapsed = time.time() - self.phase3_start_time
                cmd.linear.x = self.final_forward_speed
                cmd.angular.z = float(raw_omega)

                if elapsed >= self.final_forward_duration:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.phase = "DONE"
                    self.get_logger().info("🏁 ALIGN 미션 완료!")

                    # ALIGN 미션 성공 처리
                    self.finish_align_mission(succeeded=True)

        else:
            # 라인 미검출
            now = time.time()

            # 🔹 4초 타임아웃용 시작 시각 기록
            if self.no_line_start_time is None:
                self.no_line_start_time = now
            elapsed_no_line = now - self.no_line_start_time

            # 🔹 4초 이상 라인을 못 보면 그냥 ALIGN 미션 통과 처리 (다음 페이즈로 넘어감)
            if elapsed_no_line >= 4.0:
                self.get_logger().warn(
                    f"⏱ {elapsed_no_line:.1f}s 동안 라인 미검출 → ALIGN 미션을 통과 처리하고 다음으로 진행."
                )
                self.finish_align_mission(succeeded=True)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                return

            if self.phase == "ALIGN_MOVE":
                if self.scan_start_time is None:
                    self.scan_start_time = now

                elapsed_scan = now - self.scan_start_time

                status_text = "SCANNING..."
                if elapsed_scan < 2.0:
                    cmd.angular.z = -0.3
                    status_text = "SCAN: LEFT"
                elif elapsed_scan < 5.0:
                    cmd.angular.z = 0.3
                    status_text = "SCAN: RIGHT"
                elif elapsed_scan < 7.0:
                    cmd.angular.z = -0.3
                    status_text = "SCAN: CENTER"
                else:
                    cmd.angular.z = 0.0
                    status_text = "SCAN: GAVE UP"
                    # 여기서 실패 처리는 더 이상 하지 않음 (위 4초 타임아웃으로 통일)

                cmd.linear.x = 0.0
                cv2.putText(frame, status_text, (10, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        # 속도 명령 퍼블리시
        self.cmd_pub.publish(cmd)

        # 디버그 이미지는 필요하면 별도 토픽으로 내보낼 수 있음
        # (rviz에서 확인하고 싶으면 sensor_msgs/Image로 퍼블리시)
        # 여기서는 일단 생략
        # ---------------------------------------------------------
    # 미션이 전혀 없는 idle 상태가 30초 이상 지속되면 자동 도킹 (robot5만)
    # ---------------------------------------------------------
    def handle_idle_dock(self):
        now = time.time()

        # idle_start_time 초기화
        if self.idle_start_time is None:
            self.idle_start_time = now
            return

        idle_elapsed = now - self.idle_start_time

        # 이미 도킹 시도한 뒤라면 또 하지 않음
        if self.idle_dock_triggered:
            return

        # 30초 이상 완전 idle 상태면 도킹 시도 (robot5일 때만)
        if idle_elapsed >= 60.0:
            # # robot5만 자동 도킹
            # if self.robot_name != "robot5":
            #     # robot1은 그냥 idle 유지
            #     self.get_logger().info(
            #         f"idle {idle_elapsed:.1f}s (robot={self.robot_name}) → 자동 도킹은 robot5에만 적용."
            #     )
            #     self.idle_dock_triggered = True
            #     return

            # 이미 도킹되어 있으면 또 도킹하지 않음
            if self.is_docked:
                self.get_logger().info(
                    f"idle {idle_elapsed:.1f}s, 이미 도킹 상태입니다. (robot={self.robot_name})"
                )
                self.idle_dock_triggered = True
                return

            self.get_logger().info(
                f"🚗 idle {idle_elapsed:.1f}s → 자동 도킹 시작 (robot={self.robot_name})"
            )
            
            self.start_dock()
            self.idle_dock_triggered = True

    def publish_robot5_initial_pose(self):
        """
        robot5가 도킹된 상태일 때 한 번만 호출해서
        /robot5/initialpose 로 초기 위치를 설정.
        좌표(x, y, yaw_deg)는 주차장 맵 기준으로 네가 쓰는 값으로 바꿔.
        """
        # 👉 여기 값을 너 주차장 맵 기준으로 수정해줘
        x = -0.09684      # 예시: robot5 도킹 위치 x
        y =  4.2861     # 예시: robot5 도킹 위치 y
        yaw_deg = 0 # 예시: 정면이 아래쪽이면 180도 같은 식으로

        yaw = math.radians(yaw_deg)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # yaw → quaternion (z, w만 사용)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 적당한 covariance (너무 신뢰 낮지 않게)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25    # x
        msg.pose.covariance[7] = 0.25    # y
        msg.pose.covariance[35] = 0.068  # yaw

        self.initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"✅ Initial pose published for robot5: x={x:.2f}, y={y:.2f}, yaw={yaw_deg}deg"
        )


    # ---------------------------------------------------------
    # 🔹 비동기 언도킹 시작
    # ---------------------------------------------------------
    def start_undock(self):
        """언도킹을 비동기로 시작합니다."""
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Undock action server not available!")
            # 언도킹 실패해도 네비게이션 시도
            self.is_docked = False
            return
        
        self.get_logger().info("📤 Undocking...")
        goal_msg = Undock.Goal()
        
        send_goal_future = self.undock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.undock_goal_response_callback)
        
        self.undocking_in_progress = True

    def undock_goal_response_callback(self, future):
        """언도킹 goal 응답 콜백"""
        self.undock_goal_handle = future.result()
        
        if self.undock_goal_handle is None:
            self.get_logger().error("❌ Undock goal_handle이 None입니다!")
            self.undocking_in_progress = False
            self.is_docked = False  # 실패해도 네비게이션 시도
            return
            
        if not self.undock_goal_handle.accepted:
            self.get_logger().error("❌ Undock goal rejected!")
            self.undocking_in_progress = False
            self.is_docked = False
            return
        
        self.get_logger().info("✅ Undock goal accepted, waiting for result...")
        self.undock_result_future = self.undock_goal_handle.get_result_async()
        self.undock_result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        """언도킹 결과 콜백"""
        result = future.result()
        
        if result is None:
            self.get_logger().error("❌ Undock result가 None입니다!")
            self.is_docked = False
        elif result.result.is_docked:
            self.get_logger().error("❌ Undock failed - still docked")
            self.is_docked = True
        else:
            self.get_logger().info("✅ Undock successful!")
            self.is_docked = False
        
        self.undocking_in_progress = False
        self.undock_goal_handle = None
        self.undock_result_future = None

        # 🔥 여기 추가
        # 현재 수행해야 할 미션이 남아 있으면, 언도킹 직후 곧바로 Nav2 시작
        if self.current_mission is not None:
            self.get_logger().info(
                f"📍 Undock 완료 → 현재 미션 {self.current_mission.mission_id}에 대해 Nav2 시작"
            )
            # is_docked는 이미 False라서, 이번에는 undock 안 하고 바로 네비게이션으로 들어감
            self.send_nav_goal_for_mission(self.current_mission)

    def check_undock_result(self):
        """언도킹 진행 상태 체크 (타이머에서 호출)"""
        # 언도킹이 완료되면 자동으로 콜백이 처리하므로 여기서는 특별한 작업 불필요
        # 타임아웃 처리만 추가
        pass

    # ---------------------------------------------------------
    # 🔹 비동기 도킹 시작
    # ---------------------------------------------------------
    def start_dock(self):
        """도킹을 비동기로 시작합니다."""
        if not self.dock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Dock action server not available!")
            return False
        
        self.get_logger().info("📥 Docking...")
        goal_msg = Dock.Goal()
        
        send_goal_future = self.dock_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.dock_goal_response_callback)
        
        self.docking_in_progress = True
        return True

    def dock_goal_response_callback(self, future):
        """도킹 goal 응답 콜백"""
        self.dock_goal_handle = future.result()
        
        if self.dock_goal_handle is None:
            self.get_logger().error("❌ Dock goal_handle이 None입니다!")
            self.docking_in_progress = False
            return
            
        if not self.dock_goal_handle.accepted:
            self.get_logger().error("❌ Dock goal rejected!")
            self.docking_in_progress = False
            return
        
        self.get_logger().info("✅ Dock goal accepted, waiting for result...")
        self.dock_result_future = self.dock_goal_handle.get_result_async()
        self.dock_result_future.add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future):
        """도킹 결과 콜백"""
        result = future.result()
        
        if result is None:
            self.get_logger().error("❌ Dock result가 None입니다!")
            self.is_docked = False
        elif result.result.is_docked:
            self.get_logger().info("✅ Dock successful!")
            self.is_docked = True
            
            # robot5가 도킹 성공하면 initial pose 설정
            if self.robot_name == "robot5":
                self.get_logger().info("📍 robot5 docked. Publishing initial pose...")
                self.publish_robot5_initial_pose()
        else:
            self.get_logger().error("❌ Dock failed")
            self.is_docked = False
        
        self.docking_in_progress = False
        self.dock_goal_handle = None
        self.dock_result_future = None

    def check_dock_result(self):
        """도킹 진행 상태 체크 (타이머에서 호출)"""
        # 도킹이 완료되면 자동으로 콜백이 처리하므로 여기서는 특별한 작업 불필요
        pass

    # ---------------------------------------------------------
    # ALIGN 미션 종료 처리 (성공/실패 공통)
    # ---------------------------------------------------------
    def finish_align_mission(self, succeeded: bool):
        if self.current_mission is None:
            return

        mission_id = self.current_mission.mission_id

        if succeeded:
            self.publish_status(
                mission_id,
                'SUCCEEDED',
                1.0,
                'Align finished'
            )
        else:
            self.publish_status(
                mission_id,
                'FAILED',
                0.0,
                'Align failed (no line)'
            )

        # 상태 리셋
        self.align_active = False
        self.enable_line_detect = False
        self.phase = "IDLE"

        # 현재 미션 종료 → 다음 미션으로 진행 가능
        self.current_mission = None


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
