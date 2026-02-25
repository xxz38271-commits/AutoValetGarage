import uuid
from datetime import datetime
from typing import Optional, List, Tuple
from dataclasses import dataclass

import time

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

from supabase import create_client, Client


@dataclass
class ParkingSpace:
    """주차 공간 데이터"""
    location_id: str
    zone: str
    x: float
    y: float
    orientation: str
    is_occupied: bool


class VehicleEntryController(Node):
    """통합 입차 관리 노드"""
    
    def __init__(self):
        super().__init__('vehicle_entry_controller')
        
        # ==================== 시간 제어 변수 ====================
        self.last_task_created_time = 0.0  # 마지막 Task 생성 시간
        self.cooldown_seconds = 10.0  # 10초 쿨다운
        self.last_detection_time = 0.0  # 마지막 YOLO 실행 시간
        self.detection_interval = 0.1  # 0.1초마다 감지 (10fps)
        
        # ==================== 상태 제어 변수 ====================
        self.is_processing = False  # Task 처리 중 플래그
        self.enabled = True
        self.last_detected_label = None
        
        # ==================== 감지 설정 ====================
        self.min_confidence = 0.77  # 최소 신뢰도 임계값
        
        # ==================== Supabase 연결 ====================
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')
        
        supabase_url = self.get_parameter('supabase_url').value
        supabase_key = self.get_parameter('supabase_key').value
        
        try:
            self.supabase: Client = create_client(supabase_url, supabase_key)
            self.get_logger().info("✅ Supabase 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ Supabase 연결 실패: {e}")
            raise
        
        # ==================== YOLO 모델 로드 ====================
        model_path = "/home/rokey/ros2_ws/src/main_project/best.pt"
        self.model = YOLO(model_path)
        
        # CUDA 설정
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("✅ CUDA 사용")
        else:
            self.get_logger().info("⚠️ CPU 사용")
        
        # 레이블 맵
        self.label_map = {
            'big': 'C',
            'mid': 'B',
            'small': 'A'
        }
        
        # ==================== 기타 변수 ====================
        self.parking_spaces_cache: List[ParkingSpace] = []
        self.bridge = CvBridge()
        
        # ==================== ROS2 통신 ====================
        # Publisher
        self.image_pub = self.create_publisher(Image, '/yolo/debug_image', 10)
        self.task_created_pub = self.create_publisher(String, '/task_created', 10)
        
        # Subscriber
        self.enable_sub = self.create_subscription(
            Bool,
            '/yolo/enable',
            self.enable_callback,
            10
        )
        
        # ==================== 웹캠 설정 ====================
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 웹캠을 열 수 없습니다")
            raise RuntimeError("웹캠 초기화 실패")
        
        # ==================== 초기화 ====================
        self.refresh_parking_data()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚗 Vehicle Entry Controller 시작! (개선 버전)")
        self.get_logger().info(f"   - 차량 분류: {self.label_map}")
        self.get_logger().info(f"   - 최소 신뢰도: {self.min_confidence}")
        self.get_logger().info(f"   - 감지 간격: {self.detection_interval}초 (10fps)")
        self.get_logger().info(f"   - 쿨다운: {self.cooldown_seconds}초")
        self.get_logger().info(f"   - 주차 공간: {len(self.parking_spaces_cache)}개")
        self.get_logger().info("=" * 60)
    
    # ==================== YOLO 감지 ====================
    
    def enable_callback(self, msg: Bool):
        """YOLO 활성화/비활성화"""
        self.enabled = msg.data
        if not self.enabled:
            self.last_detected_label = None
            self.is_processing = False  # 비활성화 시 처리 플래그도 초기화
        status = "활성화" if self.enabled else "비활성화"
        self.get_logger().info(f"🔄 YOLO {status}")
    
    def run_detection(self):
        """
        YOLO 감지 실행 (3단계 방어)
        1. 시간 간격 체크 (10fps 제어)
        2. 활성화 & 처리 중 체크
        3. YOLO 추론 및 감지
        """
        current_time = time.time()
        
        # ===== 1단계: 감지 간격 체크 (FPS 제어) =====
        if current_time - self.last_detection_time < self.detection_interval:
            return
        
        self.last_detection_time = current_time
        
        # ===== 2단계: 활성화 & 처리 중 체크 =====
        if not self.enabled:
            return
        
        if self.is_processing:
            # 처리 중에는 디버그 이미지만 발행 (감지는 스킵)
            ret, frame = self.cap.read()
            if ret:
                detection_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_pub.publish(detection_img_msg)
            return
        
        # ===== 3단계: YOLO 추론 =====
        # 이미지 캡처
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("❌ 웹캠 이미지 캡처 실패")
            return
        
        # YOLO 추론 (confidence 임계값 적용)
        results = self.model(frame, verbose=False, conf=self.min_confidence)
        annotated_image = results[0].plot()
        
        detections = results[0].boxes
        
        current_best_label = None
        max_conf = 0.0
        raw_label = None
        
        # 가장 신뢰도 높은 객체 선택
        if len(detections) > 0:
            for box in detections:
                conf = float(box.conf[0])
                
                if conf > max_conf:
                    max_conf = conf
                    cls = int(box.cls[0])
                    raw_label = self.model.names[cls]  # 'big', 'mid', 'small'
                    current_best_label = self.label_map.get(raw_label, raw_label)
        
        # 새로운 차량 감지 (상태 변경)
        if current_best_label is not None and current_best_label != self.last_detected_label:
            self.get_logger().info(f"\n🔔 새로운 차량 감지!")
            self.get_logger().info(f"   - 원본: {raw_label}")
            self.get_logger().info(f"   - 분류: {current_best_label}")
            self.get_logger().info(f"   - 신뢰도: {max_conf:.2f}")
            
            # 처리 플래그 설정
            self.is_processing = True
            
            # 🎯 전체 프로세스 시작
            self.process_vehicle_entry(current_best_label, max_conf)
            
            # 처리 완료 후 플래그 해제
            self.is_processing = False
            
            # 상태 업데이트
            self.last_detected_label = current_best_label
        
        elif current_best_label is None:
            # 객체 사라짐
            if self.last_detected_label is not None:
                self.get_logger().info("💨 객체 사라짐")
            self.last_detected_label = None
        
        # 디버그 이미지 발행
        detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        self.image_pub.publish(detection_img_msg)
    
    # ==================== 주차 공간 관리 ====================
    
    def refresh_parking_data(self):
        """DB에서 주차 공간 데이터 새로고침"""
        try:
            response = self.supabase.table('parking_locations').select('*').execute()
            
            self.parking_spaces_cache = []
            for row in response.data:
                space = ParkingSpace(
                    location_id=row['location_id'],
                    zone=row['zone'],
                    x=float(row['x']),
                    y=float(row['y']),
                    orientation=str(row['orientation']),
                    is_occupied=row.get('is_occupied', False)
                )
                self.parking_spaces_cache.append(space)
            
            self.get_logger().info(f"📊 주차 공간 데이터 로드: {len(self.parking_spaces_cache)}개")
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 조회 실패: {e}")
    
    def allocate_parking_space(self, vehicle_type: str) -> Optional[ParkingSpace]:
        """
        주차 공간 할당
        
        우선순위:
        1. vehicle_type에 맞는 zone 우선 (A→A존, B→B존, C→C존)
        2. 안쪽(_1) > 바깥쪽(_2) 
        3. 큰 숫자 우선 (4>3>2>1)
        
        Args:
            vehicle_type: 'A', 'B', 'C'
        
        Returns:
            할당된 ParkingSpace 또는 None
        """
        # 1. DB 최신 데이터 가져오기
        self.refresh_parking_data()
        
        # 2. 빈 공간 필터링 (X_n_1, X_n_2 형태만)
        available_spaces = [
            space for space in self.parking_spaces_cache
            if not space.is_occupied 
            and '_' in space.location_id 
            and len(space.location_id.split('_')) == 3
        ]
        
        if not available_spaces:
            self.get_logger().error("❌ 사용 가능한 주차 공간이 없습니다")
            return None
        
        # 3. 선호 zone 우선 정렬
        preferred_zone = vehicle_type.upper()
        preferred_spaces = [s for s in available_spaces if s.zone == preferred_zone]
        other_spaces = [s for s in available_spaces if s.zone != preferred_zone]
        sorted_spaces = preferred_spaces + other_spaces
        
        self.get_logger().info(f"🎯 {vehicle_type}타입 → {preferred_zone}존 우선 ({len(preferred_spaces)}개 가능)")
        
        # 4. 우선순위 정렬
        def get_priority_key(space: ParkingSpace) -> Tuple[int, int]:
            parts = space.location_id.split('_')
            if len(parts) != 3:
                return (999, 999)
            
            zone_num = int(parts[1])  # A_2_1 → 2
            position = int(parts[2])  # A_2_1 → 1
            
            # 안쪽(_1) 우선, 큰 숫자 우선
            return (position, -zone_num)  # position 작을수록 우선 (1<2)
        
        sorted_spaces.sort(key=get_priority_key)
        
        # 5. 최우선 공간 선택
        selected = sorted_spaces[0]
        
        self.get_logger().info(f"✅ 주차 공간 할당: {selected.location_id} (좌표: {selected.x}, {selected.y})")
        
        return selected
    
    def update_parking_status(self, location_id: str, is_occupied: bool, vehicle_type: str = None):
        """DB에 주차 상태 업데이트"""
        try:
            update_data = {
                'is_occupied': is_occupied,
            }
            
            self.supabase.table('parking_locations').update(update_data).eq(
                'location_id', location_id
            ).execute()
            
            status = '주차됨' if is_occupied else '비어있음'
            self.get_logger().info(f"📝 DB 업데이트: {location_id} → {status}")
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 업데이트 실패: {e}")
    
    # ==================== DB 중복 체크 ====================
    
    def check_pending_entry_task(self) -> bool:
        """
        이미 처리 중인 입차 Task가 있는지 확인
        
        Returns:
            True: 대기 중인 Task가 있음 (새로 생성 금지)
            False: 대기 중인 Task가 없음 (새로 생성 가능)
        """
        try:
            response = self.supabase.table('tasks').select('task_id').eq(
                'task_type', 'ENTER'
            ).eq(
                'status', 'pending'
            ).eq(
                'done', False
            ).execute()
            
            if len(response.data) > 0:
                self.get_logger().warn(f"⚠️ 이미 {len(response.data)}개의 입차 Task가 대기 중입니다")
                return True
            return False
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 조회 실패: {e}")
            return True  # 에러 시 안전하게 차단
    
    # ==================== Task 생성 ====================
    
    def create_entry_task(self, vehicle_type: str, parking_spot: ParkingSpace) -> bool:
        """
        입차 Task 생성 및 DB 저장
        
        Args:
            vehicle_type: 'A', 'B', 'C'
            parking_spot: 할당된 주차 공간
        
        Returns:
            성공 여부
        """
        try:
            # 임시 차량 번호 생성 (실제로는 번호판 인식 필요)
            vehicle_plate = f"TEMP_{uuid.uuid4().hex[:6].upper()}"
            
            # Task 데이터 생성
            task_data = {
                'task_id': str(uuid.uuid4()),
                'task_type': 'ENTER',
                'vehicle_plate': vehicle_plate,
                'vehicle_type': vehicle_type,
                'assigned_robot': 'robot1',  # 입차는 robot1 우선
                'start_location': 'ENTRANCE',
                'target_location': parking_spot.location_id,
                'status': 'pending',
                'done': False,
                'priority': 60,
                'created_at': datetime.utcnow().isoformat()
            }
            
            # DB에 저장
            response = self.supabase.table('tasks').insert(task_data).execute()
            
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("🎉 입차 Task 생성 완료!")
            self.get_logger().info(f"   - Task ID: {task_data['task_id']}")
            self.get_logger().info(f"   - 차량 타입: {vehicle_type}")
            self.get_logger().info(f"   - 차량 번호: {vehicle_plate}")
            self.get_logger().info(f"   - 주차 위치: {parking_spot.location_id}")
            self.get_logger().info(f"   - 할당 로봇: robot1")
            self.get_logger().info(f"   - 상태: pending")
            self.get_logger().info("=" * 60 + "\n")
            
            # Task 생성 알림 발행
            task_msg = String()
            task_msg.data = f"ENTER|{vehicle_type}|{parking_spot.location_id}|{vehicle_plate}"
            self.task_created_pub.publish(task_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Task 생성 실패: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
    
    # ==================== 전체 프로세스 ====================
    
    def process_vehicle_entry(self, vehicle_type: str, confidence: float):
        """
        차량 입차 전체 프로세스 (4단계 방어)
        1. Cooldown 체크
        2. DB 중복 체크
        3. 주차 공간 할당 및 Task 생성
        4. 성공 시 시간 업데이트
        
        Args:
            vehicle_type: 분류된 차종 ('A', 'B', 'C')
            confidence: YOLO 신뢰도
        """
        current_time = time.time()
        
        # ===== 1단계: Cooldown 체크 =====
        if current_time - self.last_task_created_time < self.cooldown_seconds:
            remaining = self.cooldown_seconds - (current_time - self.last_task_created_time)
            self.get_logger().warn(f"⏰ 쿨다운 중! {remaining:.1f}초 남음")
            return
        
        # ===== 2단계: DB 중복 체크 =====
        if self.check_pending_entry_task():
            self.get_logger().warn("⏸️ 대기 중인 입차 Task가 있어 건너뜁니다")
            return
        
        # ===== 3단계: 프로세스 실행 =====
        self.get_logger().info("\n" + "🚀" * 30)
        self.get_logger().info("입차 프로세스 시작")
        self.get_logger().info("🚀" * 30)
        
        # Step 1: 주차 공간 할당
        self.get_logger().info("\n[Step 1] 주차 공간 할당 중...")
        parking_spot = self.allocate_parking_space(vehicle_type)
        
        if not parking_spot:
            self.get_logger().error("❌ 주차 공간 할당 실패 - 프로세스 중단")
            return
        
        # Step 2: 주차 상태 업데이트 (예약)
        self.get_logger().info("\n[Step 2] 주차 상태 업데이트 중...")
        self.update_parking_status(parking_spot.location_id, True, vehicle_type)
        
        # Step 3: 입차 Task 생성
        self.get_logger().info("\n[Step 3] 입차 Task 생성 중...")
        success = self.create_entry_task(vehicle_type, parking_spot)
        
        # ===== 4단계: 성공 시 시간 업데이트 =====
        if success:
            self.last_task_created_time = current_time  # ✅ 중요: 시간 업데이트
            self.get_logger().info(f"\n✅ 입차 프로세스 완료! (다음 입차 가능: {self.cooldown_seconds}초 후)")
        else:
            self.get_logger().error("\n❌ Task 생성 실패")
            # 실패 시 주차 상태 롤백
            self.update_parking_status(parking_spot.location_id, False)
        
        self.get_logger().info("🏁" * 30 + "\n")
    
    # ==================== 종료 ====================
    
    def destroy(self):
        """리소스 정리"""
        if hasattr(self, 'cap'):
            self.cap.release()
        self.get_logger().info("🛑 리소스 정리 완료")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VehicleEntryController()
        
        # 메인 루프
        while rclpy.ok():
            node.run_detection()  # YOLO 감지 실행
            rclpy.spin_once(node, timeout_sec=0.01)  # ROS2 콜백 처리
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"❌ 에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()