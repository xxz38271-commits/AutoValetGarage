import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import json
import time
import math
import numpy as np
from typing import Optional, Dict

from builtin_interfaces.msg import Duration

# ROS Msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from irobot_create_msgs.action import Dock

# Supabase & CV
from supabase import create_client, Client
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage

# TurtleBot4 Helper
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions, TaskResult


class Robot1Node(Node):
    """Robot1 전용 노드 - ENTER Task만 처리"""
    
    def __init__(self):
        super().__init__('robot1_node')
        
        # ==================== 설정 ====================
        self.robot_id = 'robot1'
        
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')
        
        # ⭐ 초기 위치 (Home Position) 설정
        self.declare_parameter('home_x', -0.06)
        self.declare_parameter('home_y', 0.02)
        self.declare_parameter('home_orientation', 'NORTH')
        
        self.home_position = {
            'x': self.get_parameter('home_x').value,
            'y': self.get_parameter('home_y').value,
            'orientation': self.get_parameter('home_orientation').value
        }
        
        # Supabase 연결
        try:
            url = self.get_parameter('supabase_url').value
            key = self.get_parameter('supabase_key').value
            self.supabase: Client = create_client(url, key)
            self.get_logger().info("✅ Supabase 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ Supabase 연결 실패: {e}")
            raise
        
        # ==================== ROS 통신 ====================
        self.callback_group = ReentrantCallbackGroup()
        
        # Task 구독
        self.task_sub = self.create_subscription(
            String,
            f'/task_command/{self.robot_id}',
            self.task_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Audio Publisher
        self.audio_pub = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        
        # Vision (Line Alignment)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage,
            f'/{self.robot_id}/oakd/rgb/image_raw/compressed',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        
        # Navigator
        self.navigator = TurtleBot4Navigator()
        
        # ==================== 상태 변수 ====================
        self.is_busy = False
        self.current_task = None
        self.last_task_time = time.time()
        
        # Vision 상태
        self.vision_enabled = False
        self.alignment_stable_count = 0
        
        # Idle 체크 타이머 (30초마다)
        self.create_timer(30.0, self.check_idle_status, callback_group=self.callback_group)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"🤖 Robot1 Node 시작!")
        self.get_logger().info("   - ENTER Task 전용 (입차만 처리)")
        self.get_logger().info(f"   - Home: ({self.home_position['x']:.2f}, {self.home_position['y']:.2f}, {self.home_position['orientation']})")
        self.get_logger().info("=" * 60)
    
    
    def task_callback(self, msg: String):
        """Task 명령 수신"""
        if self.is_busy:
            self.get_logger().warn("⚠️ 이미 작업 중 - Task 무시")
            return
        
        try:
            task = json.loads(msg.data)
            
            # ENTER Task만 처리
            if task['task_type'] != 'ENTER':
                self.get_logger().warn(f"⚠️ 지원하지 않는 Task: {task['task_type']} (ENTER만 처리)")
                return
            
            self.get_logger().info("\n" + "🔔" * 30)
            self.get_logger().info(f"Task 수신: {task['task_type']}")
            self.get_logger().info("🔔" * 30)
            
            # Task 상태를 'assigned'로 변경
            self.mark_task_assigned(task['task_id'])
            
            # Task 실행
            self.execute_task(task)
            
        except Exception as e:
            self.get_logger().error(f"❌ Task 처리 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.is_busy = False
    
    def execute_task(self, task: Dict):
        """Task 실행"""
        self.is_busy = True
        self.current_task = task
        self.last_task_time = time.time()
        
        try:
            self.do_enter(task)
        
        except Exception as e:
            self.get_logger().error(f"❌ Task 실행 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        finally:
            self.is_busy = False
            self.current_task = None
            self.last_task_time = time.time()
    
    # ==================== ENTER Task ====================
    
    def do_enter(self, task: Dict):
        """
        ⭐ 입차 작업
        
        경로 A (Waypoint 있음):
        Start → Waypoint → Line Align → Target → 180° → Waypoint → (Idle시) Home
        
        경로 B (Waypoint 없음):
        Start → Target → 180° → (Idle시) Home
        """
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("🚗 입차 작업 시작")
        self.get_logger().info("=" * 60)
        
        vehicle_plate = task['vehicle_plate']
        start_coords = task['start_coords']
        target_coords = task['target_coords']
        
        # ⭐ Waypoint 확인 (키 이름: target_waypoint_coords)
        has_waypoint = 'target_waypoint_coords' in task and task['target_waypoint_coords'] is not None
        
        # Task 정보 출력
        self.get_logger().info(f"   차량: {vehicle_plate}")
        self.get_logger().info(f"   출발: {task['start_location']} {start_coords}")
        
        if has_waypoint:
            waypoint_coords = task['target_waypoint_coords']
            self.get_logger().info(f"   경유: {task.get('target_waypoint_location', 'WAYPOINT')} {waypoint_coords} ⭐")
            self.get_logger().info("   → 라인 정렬 수행!")
        else:
            self.get_logger().info("   경유: 없음 (직진)")
        
        self.get_logger().info(f"   도착: {task['target_location']} {target_coords}")
        
        # ==================== 작업 시작 ====================
        
        # 1. Undock
        self.get_logger().info("\n[1/?] Undocking...")
        self.undock()
        
        # 2. Start (ENTRANCE)로 이동
        self.get_logger().info(f"\n[2/?] 입구로 이동: {task['start_location']}")
        if not self.nav_to_coords(start_coords):
            self.handle_navigation_failure(task['task_id'])
            return
        
        # 3. 차량 Pick
        self.get_logger().info(f"\n[3/?] 차량 Pick: {vehicle_plate}")
        self.perform_pick_action()
        
        # ⭐ 경로 분기
        if has_waypoint:
            # ========== 경로 A: Waypoint 경유 ==========
            
            # 4. Waypoint로 이동
            self.get_logger().info(f"\n[4/9] Waypoint로 이동")
            if not self.nav_to_coords(waypoint_coords):
                self.handle_navigation_failure(task['task_id'])
                return
            
            # 5. Line Align (비전 정렬)
            self.get_logger().info("\n[5/9] 👁️ 라인 정렬 시작...")
            self.perform_visual_alignment()
            self.get_logger().info("   ✅ 라인 정렬 완료!")
            
            # 6. Target (주차 공간)으로 이동
            self.get_logger().info(f"\n[6/9] 주차 공간으로 직진")
            if not self.nav_to_coords(target_coords):
                self.handle_navigation_failure(task['task_id'])
                return
            
            # 7. 차량 Place
            self.get_logger().info("\n[7/9] 차량 Place")
            self.perform_place_action()
            
            # 8. 180도 회전 (복귀 준비)
            self.get_logger().info("\n[8/9] 180도 회전...")
            self.perform_180_turn(target_coords)
            
            # 9. Waypoint로 복귀
            self.get_logger().info(f"\n[9/9] Waypoint로 복귀")
            if not self.nav_to_coords(waypoint_coords):
                self.get_logger().warn("⚠️ Waypoint 복귀 실패 - 계속 진행")

            self.return_to_home()
            
        
        else:
            # ========== 경로 B: Waypoint 없음 (직진) ==========
            
            # 4. Target (주차 공간)으로 직진
            self.get_logger().info(f"\n[4/6] 주차 공간으로 이동")
            if not self.nav_to_coords(target_coords):
                self.handle_navigation_failure(task['task_id'])
                return
            
            # 5. 차량 Place
            self.get_logger().info("\n[5/6] 차량 Place")
            self.perform_place_action()
            
            # 6. 180도 회전 (복귀 준비)
            self.get_logger().info("\n[6/6] 180도 회전...")
            self.perform_180_turn(target_coords)
        
        # ==================== 작업 완료 ====================
        
        self.get_logger().info("\n✅ Task 완료!")
        self.mark_task_done(task['task_id'])
        
        self.get_logger().info("\n✅ 입차 작업 완료!")
        self.get_logger().info("=" * 60 + "\n")
    
    def perform_180_turn(self, current_coords: Dict):
        """
        현재 위치에서 180도 회전
        현재 방향의 반대 방향으로 향하도록 내비게이션
        """
        # 현재 orientation의 반대 방향 계산
        orientation_map = {
            'NORTH': 'SOUTH',
            'SOUTH': 'NORTH',
            'EAST': 'WEST',
            'WEST': 'EAST'
        }
        
        current_orientation = current_coords['orientation']
        opposite_orientation = orientation_map.get(current_orientation, 'SOUTH')
        
        # 현재 위치에서 방향만 바꿔서 내비게이션
        turn_coords = {
            'x': current_coords['x'],
            'y': current_coords['y'],
            'orientation': opposite_orientation
        }
        
        self.get_logger().info(f"   🔄 {current_orientation} → {opposite_orientation}")
        
        # 제자리에서 방향 전환 (Nav2 이용)
        if not self.nav_to_coords(turn_coords):
            self.get_logger().warn("⚠️ 180도 회전 실패 - 계속 진행")
    
    # ==================== Navigation ====================
    
    def nav_to_coords(self, coords: Dict) -> bool:
        """Nav2를 이용한 이동"""
        x, y = coords['x'], coords['y']
        yaw = self.get_yaw_from_orientation(coords['orientation'])
        
        self.get_logger().info(f"   🚶 Navigate → ({x:.2f}, {y:.2f}, {coords['orientation']})")
        
        try:
            goal_pose = self.navigator.getPoseStamped([x, y], yaw)
            self.navigator.goToPose(goal_pose)
            
            while not self.navigator.isTaskComplete():
                time.sleep(0.5)
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("   ✅ 도착!")
                return True
            else:
                self.get_logger().error(f"   ❌ 이동 실패! (result: {result})")
                return False
        
        except Exception as e:
            self.get_logger().error(f"   ❌ Navigation 에러: {e}")
            return False
    
    def get_yaw_from_orientation(self, orientation: str) -> float:
        """방향 문자열을 yaw 각도로 변환"""
        mapping = {
            'NORTH': TurtleBot4Directions.NORTH,
            'SOUTH': TurtleBot4Directions.SOUTH,
            'EAST': TurtleBot4Directions.EAST,
            'WEST': TurtleBot4Directions.WEST
        }
        return mapping.get(orientation.upper(), TurtleBot4Directions.NORTH)
    
    # ==================== 작업 동작 ====================
    
    def perform_pick_action(self):
        """Pick 동작: 비프음 + 3초 대기"""
        self.get_logger().info("   🤖 Pick 시작...")
        
        # 3초간 비프음
        for _ in range(3):
            self.play_beep()
            time.sleep(1.0)
        
        self.get_logger().info("   ✅ Pick 완료!")
    
    def perform_place_action(self):
        """Place 동작: 비프음 + 2초 대기"""
        self.get_logger().info("   🤖 Place 시작...")
        
        # 2초간 비프음
        for _ in range(2):
            self.play_beep()
            time.sleep(1.0)
        
        self.get_logger().info("   ✅ Place 완료!")
    
    def play_beep(self):
        """Create3 Audio Note 발행"""
        msg = AudioNoteVector()
        msg.append = False
        duration = Duration()
        duration.sec = 0
        duration.nanosec = 400_000_000  # 0.4초
        # 삐-뽀 (High note, Low note)
        note1 = AudioNote(frequency=800, max_runtime=duration)
        note2 = AudioNote(frequency=600, max_runtime=duration)
        msg.notes = [note1, note2]
        
        self.audio_pub.publish(msg)
    
    # ==================== Vision Alignment ====================
    
    def perform_visual_alignment(self):
        """
        비전 정렬 수행
        - 라인을 따라 전진하면서 중앙 정렬
        - 15초 타임아웃
        """
        self.vision_enabled = True
        self.alignment_stable_count = 0
        
        start_time = time.time()
        
        while self.vision_enabled:
            if time.time() - start_time > 15.0:
                self.get_logger().warn("⚠️ 정렬 시간 초과!")
                self.vision_enabled = False
                break
            
            time.sleep(0.1)
        
        # 정렬 후 멈춤
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def image_callback(self, msg: CompressedImage):
        """
        ⭐ Vision Alignment Logic (컨투어 필터링 + 모멘트)
        - 면적/종횡비 필터링으로 노이즈 제거
        - 화면 중앙에 가장 가까운 라인 선택
        - 라인 못 찾으면 좌우 스캔
        """
        if not self.vision_enabled:
            return
        
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
            h, w, _ = frame.shape
            
            # ROI: 화면 하단 1/3
            roi_y_start = int(h * 2 / 3)
            roi = frame[roi_y_start:h, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 흰색 라인 검출 (타이트하게)
            lower_white = np.array([0, 0, 200], dtype=np.uint8)
            upper_white = np.array([179, 30, 255], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # ⭐ 노이즈 제거 (모폴로지 연산)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 작은 점 제거
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 끊김 메움
            
            # ⭐ 컨투어 기반 라인 검출
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            line_found = False
            lane_center_x = None
            
            if contours:
                # 필터링 파라미터
                min_area = 300.0      # 최소 면적
                min_aspect = 2.0      # 최소 종횡비 (세로/가로)
                
                candidates = []
                for c in contours:
                    area = cv2.contourArea(c)
                    if area < min_area:
                        continue
                    
                    x, y, cw, ch = cv2.boundingRect(c)
                    aspect = ch / float(cw + 1e-3)
                    if aspect < min_aspect:  # 라인 형태 아님
                        continue
                    
                    candidates.append((area, c))
                
                # ⭐ 화면 중앙에 가장 가까운 후보 선택
                if candidates:
                    image_center_x = w / 2.0
                    best_c = None
                    min_distance = float('inf')
                    
                    for area, c in candidates:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cx = M["m10"] / M["m00"]
                            dist = abs(cx - image_center_x)
                            
                            if dist < min_distance:
                                min_distance = dist
                                best_c = c
                    
                    # ⭐ 최종 선택된 컨투어의 중심 계산
                    if best_c is not None:
                        M = cv2.moments(best_c)
                        if M["m00"] > 0:
                            lane_center_x = M["m10"] / M["m00"]
                            line_found = True
            
            # ==================== 제어 로직 ====================
            cmd = Twist()
            
            if line_found and lane_center_x is not None:
                # ✅ 라인 발견 - 스캔 타이머 초기화
                if not hasattr(self, 'scan_start_time'):
                    self.scan_start_time = None
                self.scan_start_time = None
                
                # 오차 계산
                image_center_x = w / 2.0
                err = lane_center_x - image_center_x
                
                # P 제어
                k_p = 0.003
                max_angular_z = 0.5
                raw_omega = -k_p * err
                
                # 각속도 제한
                if raw_omega > max_angular_z:
                    raw_omega = max_angular_z
                elif raw_omega < -max_angular_z:
                    raw_omega = -max_angular_z
                
                cmd.linear.x = 0.05  # 천천히 전진
                
                # Dead band (작은 오차는 무시)
                if abs(err) < 5.0:
                    cmd.angular.z = 0.0
                else:
                    cmd.angular.z = float(raw_omega)
                
                # 정렬 완료 조건: 중앙 근처 안정적 유지
                if abs(err) < 50.0:  # 픽셀 단위
                    self.alignment_stable_count += 1
                    
                    # 10프레임 연속 중앙이면 완료
                    if self.alignment_stable_count > 10:
                        self.vision_enabled = False
                        self.get_logger().info("   ✅ 라인 정렬 완료!")
                else:
                    self.alignment_stable_count = 0
            
            else:
                # ❌ 라인 미검출 - 스캔 로직
                if not hasattr(self, 'scan_start_time'):
                    self.scan_start_time = None
                
                if self.scan_start_time is None:
                    self.scan_start_time = time.time()
                    self.get_logger().warn("⚠️ 라인 미검출 - 스캔 시작")
                
                elapsed_scan = time.time() - self.scan_start_time
                
                cmd.linear.x = 0.0  # 제자리에서 회전
                
                if elapsed_scan < 2.0:
                    # 0~2초: 왼쪽 보기
                    cmd.angular.z = 0.3
                elif elapsed_scan < 5.0:
                    # 2~5초: 오른쪽 보기
                    cmd.angular.z = -0.3
                elif elapsed_scan < 7.0:
                    # 5~7초: 다시 중앙 복귀
                    cmd.angular.z = 0.3
                else:
                    # 7초 넘어도 없으면 정지
                    cmd.angular.z = 0.0
                    self.get_logger().error("❌ 라인 찾기 실패 - 정렬 중단")
                    self.vision_enabled = False
                
                self.alignment_stable_count = 0
            
            self.cmd_vel_pub.publish(cmd)
        
        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")
    
    # ==================== Dock/Undock & Home ====================
    
    def undock(self):
        """Undocking"""
        if self.navigator.getDockedStatus():
            self.get_logger().info("   🔓 Undock")
            self.navigator.undock()
            time.sleep(2.0)
    
    def dock(self):
        """Docking"""
        if not self.navigator.getDockedStatus():
            self.get_logger().info("   🔒 Dock")
            self.navigator.dock()
            time.sleep(2.0)
    
    def return_to_home(self):
        """
        ⭐ 초기 위치로 복귀 후 Docking
        """
        self.get_logger().info("🏠 초기 위치로 복귀 중...")
        
        # 초기 위치로 이동
        if self.nav_to_coords(self.home_position):
            self.get_logger().info("✅ 초기 위치 도착!")
            
            # Docking 시도
            self.dock()
            self.get_logger().info("✅ Docking 완료!")
        else:
            self.get_logger().error("❌ 초기 위치 복귀 실패 - 현재 위치에서 Docking 시도")
            self.dock()
    
    # ==================== DB 업데이트 ====================
    
    def mark_task_assigned(self, task_id: str):
        """Task를 'assigned' 상태로 변경"""
        try:
            self.supabase.table('tasks').update({
                'status': 'assigned'
            }).eq('task_id', task_id).execute()
            
            self.get_logger().info(f"✅ Task 할당 완료: {task_id}")
        
        except Exception as e:
            self.get_logger().error(f"❌ Task 할당 실패: {e}")
    
    def mark_task_done(self, task_id: str):
        """Task 완료 처리"""
        try:
            self.supabase.table('tasks').update({
                'done': True,
                'status': 'done'
            }).eq('task_id', task_id).execute()
            
            self.get_logger().info(f"✅ Task 완료: {task_id}")
        
        except Exception as e:
            self.get_logger().error(f"❌ Task 완료 실패: {e}")
    
    def handle_navigation_failure(self, task_id: str):
        """Navigation 실패 처리"""
        self.get_logger().error("❌ Navigation 실패 - Task 중단")
        
        try:
            self.supabase.table('tasks').update({
                'status': 'failed'
            }).eq('task_id', task_id).execute()
        except:
            pass
        
        self.is_busy = False
    
    # ==================== Idle Management ====================
    
    def check_idle_status(self):
        """
        ⭐ 작업이 없으면 초기 위치로 복귀 후 Docking
        - 마지막 작업 후 60초 이상 경과 시
        """
        if self.is_busy:
            return
        
        # 마지막 작업 후 60초 이상 경과
        if time.time() - self.last_task_time > 60.0:
            self.get_logger().info("💤 Idle 감지 - 초기 위치로 복귀...")
            
            # 초기 위치로 복귀
            self.return_to_home()
            
            # 타이머 리셋
            self.last_task_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "=" * 60)
    print("🤖 Robot1 Node 시작")
    print("=" * 60)
    print("\n📋 담당 업무:")
    print("   - ENTER: 입차 작업만 처리")
    print("\n🛣️ 작업 흐름:")
    print("   [Waypoint 있음]")
    print("   Start → Waypoint → Line Align → Target → 180° → Waypoint → Home")
    print("\n   [Waypoint 없음]")
    print("   Start → Target → 180° → Home")
    print("\n🏠 초기 위치 설정:")
    print("   파라미터: home_x, home_y, home_orientation")
    print("\n💡 60초간 작업 없으면 자동으로 Home 복귀!")
    print("=" * 60 + "\n")
    
    node = Robot1Node()
    
    # MultiThreadedExecutor (Nav2와 구독 동시 처리)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("\n👋 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()