import json
import threading
from typing import Dict, Optional
from supabase import create_client, Client

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotManager(Node):
    """
    Robot Manager (중앙 Task 관리자) - Realtime 버전
    
    ⭐ 수정: start/target 모두 주차 위치면 waypoint 자동 추가
    """
    
    def __init__(self):
        super().__init__('robot_manager')
        
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
        
        # ==================== 좌표 캐시 (메모리) ====================
        self.location_cache: Dict[str, Dict] = {}
        self.refresh_location_cache()
        
        # ==================== ROS2 Publisher ====================
        self.robot1_pub = self.create_publisher(
            String,
            '/task_command/robot1',
            10
        )
        
        self.robot5_pub = self.create_publisher(
            String,
            '/task_command/robot5',
            10
        )
        
        # ==================== Realtime 구독 설정 ====================
        # ✅ 폴링 대신 Realtime 구독 사용
        self.setup_realtime_subscription()
        
        # 5분마다 좌표 캐시 갱신 (이건 유지)
        self.create_timer(300.0, self.refresh_location_cache)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 Robot Manager 시작! (Realtime 버전)")
        self.get_logger().info("   - Task 모니터링: Realtime 구독 (DB 부하 최소화)")
        self.get_logger().info("   - 좌표 캐싱: 활성화")
        self.get_logger().info("   - Start/Target Waypoint 자동 추가 ⭐")
        self.get_logger().info("=" * 60)
    
    # ==================== Realtime 구독 ====================
    
    def setup_realtime_subscription(self):
        """
        Supabase Realtime 구독 설정
        """
        try:
            # Realtime 채널 생성
            channel = self.supabase.channel('tasks_channel')
            
            # tasks 테이블 구독 (pending 상태만 관심)
            channel.on_postgres_changes(
                event='INSERT',  # 새 Task 추가
                schema='public',
                table='tasks',
                callback=self.handle_task_insert
            ).on_postgres_changes(
                event='UPDATE',  # Task 상태 변경
                schema='public',
                table='tasks',
                callback=self.handle_task_update
            ).subscribe()
            
            self.get_logger().info("✅ Realtime 구독 시작: tasks 테이블")
            
            # 초기 pending Task 처리 (구독 전에 이미 있던 것들)
            self.process_existing_pending_tasks()
            
        except Exception as e:
            self.get_logger().error(f"❌ Realtime 구독 실패: {e}")
            self.get_logger().warn("⚠️ 폴링 모드로 폴백합니다")
            # 폴백: 폴링 사용
            self.create_timer(5.0, self.check_for_new_tasks_polling)
    
    def handle_task_insert(self, payload):
        """
        새 Task가 INSERT되었을 때 호출
        """
        try:
            new_task = payload['new']
            
            # pending 상태만 처리
            if new_task.get('status') == 'pending':
                task_id = new_task['task_id']
                self.get_logger().info(f"\n🔔 새 Task 감지 (INSERT): {task_id}")
                
                # ROS2 콜백에서 안전하게 처리하기 위해 타이머 사용
                self.create_timer(
                    0.1,
                    lambda: self.process_task_safe(new_task),
                    one_shot=True
                )
        
        except Exception as e:
            self.get_logger().error(f"❌ Task INSERT 처리 중 에러: {e}")
    
    def handle_task_update(self, payload):
        """
        Task가 UPDATE되었을 때 호출
        """
        try:
            old_task = payload['old']
            new_task = payload['new']
            
            # 상태가 pending으로 변경된 경우만 처리
            old_status = old_task.get('status')
            new_status = new_task.get('status')
            
            if old_status != 'pending' and new_status == 'pending':
                task_id = new_task['task_id']
                self.get_logger().info(f"\n🔔 Task 재할당 감지 (UPDATE): {task_id}")
                
                self.create_timer(
                    0.1,
                    lambda: self.process_task_safe(new_task),
                    one_shot=True
                )
        
        except Exception as e:
            self.get_logger().error(f"❌ Task UPDATE 처리 중 에러: {e}")
    
    def process_task_safe(self, task: Dict):
        """
        Task를 안전하게 처리 (ROS2 타이머 콜백에서 호출)
        """
        try:
            self.process_task(task)
        except Exception as e:
            self.get_logger().error(f"❌ Task 처리 실패: {e}")
    
    def process_existing_pending_tasks(self):
        """
        구독 시작 전에 이미 pending 상태인 Task들 처리
        """
        try:
            result = self.supabase.table('tasks').select('*').eq(
                'status', 'pending'
            ).order(
                'priority', desc=True
            ).execute()
            
            if result.data:
                self.get_logger().info(f"📋 기존 pending Task {len(result.data)}개 발견")
                for task in result.data:
                    self.process_task(task)
        
        except Exception as e:
            self.get_logger().error(f"❌ 기존 Task 조회 실패: {e}")
    
    # ==================== 폴백: 폴링 모드 ====================
    
    def check_for_new_tasks_polling(self):
        """
        폴백 함수: Realtime 구독이 실패한 경우 사용
        """
        try:
            result = self.supabase.table('tasks').select('*').eq(
                'status', 'pending'
            ).order(
                'priority', desc=True
            ).execute()
            
            if result.data:
                for task in result.data:
                    self.process_task(task)
        
        except Exception as e:
            self.get_logger().error(f"❌ Task 확인 중 에러: {e}")
    
    # ==================== 좌표 캐싱 ====================
    
    def refresh_location_cache(self):
        """모든 위치 정보를 메모리에 캐싱"""
        try:
            result = self.supabase.table('parking_locations').select(
                'location_id, x, y, orientation'
            ).execute()
            
            self.location_cache = {}
            for loc in result.data:
                self.location_cache[loc['location_id']] = {
                    'x': float(loc['x']),
                    'y': float(loc['y']),
                    'orientation': loc['orientation']
                }
            
            # 특수 위치 추가
            self.location_cache['ENTRANCE'] = {
                'x': -2.99,
                'y': -0.006,
                'orientation': 'SOUTH'
            }
            self.location_cache['EXIT_ZONE'] = {
                'x': -2.93,
                'y': 4.35,
                'orientation': 'NORTH'
            }
            self.location_cache['TEMP_1'] = {
                'x': 5.0,
                'y': 15.0,
                'orientation': 'NORTH'
            }
            
            self.get_logger().info(f"📍 좌표 캐시 갱신 완료: {len(self.location_cache)}개 위치")
            
        except Exception as e:
            self.get_logger().error(f"❌ 좌표 캐시 갱신 실패: {e}")
    
    def get_coordinates(self, location_id: str) -> Optional[Dict]:
        """location_id를 좌표로 변환 (캐싱됨)"""
        if location_id in self.location_cache:
            return self.location_cache[location_id]
        
        self.get_logger().warn(f"⚠️ 좌표 정보 없음: {location_id}")
        return None
    
    # ==================== Task 처리 ====================
    
    def process_task(self, task: Dict):
        """
        Task 처리:
        1. 좌표 변환
        2. 로봇에게 명령 전송
        """
        try:
            task_id = task['task_id']
            task_type = task['task_type']
            assigned_robot = task['assigned_robot']
            
            self.get_logger().info(f"   타입: {task_type}")
            self.get_logger().info(f"   할당 로봇: {assigned_robot}")
            
            # 1. 좌표 변환
            coordinates = self.convert_task_locations(task)
            if not coordinates:
                self.get_logger().error(f"❌ 좌표 변환 실패: {task_id}")
                return
            
            # 2. Task 타입별 처리
            if task_type in ['ENTER', 'EXIT_SINGLE']:
                self.send_simple_task(task, coordinates)
            elif task_type == 'EXIT_DOUBLE':
                self.send_double_exit_task(task, coordinates)
            
        except Exception as e:
            self.get_logger().error(f"❌ Task 처리 중 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def convert_task_locations(self, task: Dict) -> Optional[Dict]:
        """
        ⭐ Task의 모든 location_id를 좌표로 변환
        
        변경사항:
        - start_location이 주차 위치면 start_waypoint 추가
        - target_location이 주차 위치면 target_waypoint 추가
        
        Returns:
            {
                'start': {...},
                'start_waypoint': {...},  # start가 주차 위치면 추가
                'target': {...},
                'target_waypoint': {...}, # target이 주차 위치면 추가
                'blocking': {...},        # EXIT_DOUBLE만
                'temp': {...}             # EXIT_DOUBLE만
            }
        """
        coordinates = {}
        
        # ==================== start_location 변환 ====================
        start_loc = task.get('start_location')
        if start_loc:
            start_coords = self.get_coordinates(start_loc)
            if not start_coords:
                return None
            coordinates['start'] = start_coords
            
            # ⭐ start가 주차 위치면 start_waypoint 추가
            if self.is_parking_location(start_loc):
                start_waypoint_loc = self.get_preparation_location(start_loc)
                start_waypoint_coords = self.get_coordinates(start_waypoint_loc)
                if start_waypoint_coords:
                    coordinates['start_waypoint'] = start_waypoint_coords
                    self.get_logger().info(f"   📍 Start Waypoint 자동 추가: {start_waypoint_loc}")
        
        # ==================== target_location 변환 ====================
        target_loc = task.get('target_location')
        if target_loc:
            target_coords = self.get_coordinates(target_loc)
            if not target_coords:
                return None
            coordinates['target'] = target_coords
            
            # ⭐ target이 주차 위치면 target_waypoint 추가
            if self.is_parking_location(target_loc):
                target_waypoint_loc = self.get_preparation_location(target_loc)
                target_waypoint_coords = self.get_coordinates(target_waypoint_loc)
                if target_waypoint_coords:
                    coordinates['target_waypoint'] = target_waypoint_coords
                    self.get_logger().info(f"   📍 Target Waypoint 자동 추가: {target_waypoint_loc}")
        
        # ==================== EXIT_DOUBLE: 추가 위치들 ====================
        if task['task_type'] == 'EXIT_DOUBLE':
            blocking_loc = task.get('blocking_location')
            if blocking_loc:
                blocking_coords = self.get_coordinates(blocking_loc)
                if blocking_coords:
                    coordinates['blocking'] = blocking_coords
                
                # blocking도 주차 위치면 waypoint 추가
                if self.is_parking_location(blocking_loc):
                    blocking_waypoint = self.get_preparation_location(blocking_loc)
                    blocking_waypoint_coords = self.get_coordinates(blocking_waypoint)
                    if blocking_waypoint_coords:
                        coordinates['blocking_waypoint'] = blocking_waypoint_coords
                        self.get_logger().info(f"   📍 Blocking Waypoint 자동 추가: {blocking_waypoint}")
            
            temp_loc = task.get('temp_location')
            if temp_loc:
                temp_coords = self.get_coordinates(temp_loc)
                if temp_coords:
                    coordinates['temp'] = temp_coords
        
        return coordinates
    
    def is_parking_location(self, location_id: str) -> bool:
        """
        주차 위치인지 확인
        
        Args:
            location_id: 'B_2_1', 'B_2' 등
        
        Returns:
            True if parking (예: B_2_1 - 언더스코어 2개)
            False if preparation (예: B_2 - 언더스코어 1개)
        """
        return location_id.count('_') == 2
    
    def get_preparation_location(self, parking_location: str) -> str:
        """
        주차 위치로부터 preparation 위치 계산
        
        Args:
            parking_location: 'B_2_1'
        
        Returns:
            preparation 위치: 'B_2'
        """
        # 마지막 언더스코어와 숫자 제거
        # 'B_2_1' → 'B_2'
        return parking_location.rsplit('_', 1)[0]
    
    # ==================== Task 전송 ====================
    
    def send_simple_task(self, task: Dict, coordinates: Dict):
        """
        ⭐ 단순 Task 전송 (ENTER, EXIT_SINGLE)
        
        변경사항: start_waypoint와 target_waypoint 모두 지원
        """
        assigned_robot = task['assigned_robot']
        
        command = {
            'task_id': task['task_id'],
            'task_type': task['task_type'],
            'vehicle_plate': task['vehicle_plate'],
            'start_location': task['start_location'],
            'start_coords': coordinates['start'],
            'target_location': task['target_location'],
            'target_coords': coordinates['target'],
            'priority': task['priority']
        }
        
        # ⭐ Start Waypoint 추가 (있으면)
        if 'start_waypoint' in coordinates:
            start_waypoint_loc = self.get_preparation_location(task['start_location'])
            command['start_waypoint_location'] = start_waypoint_loc
            command['start_waypoint_coords'] = coordinates['start_waypoint']
            self.get_logger().info(f"   ✅ Start Waypoint 포함: {start_waypoint_loc}")
        
        # ⭐ Target Waypoint 추가 (있으면)
        if 'target_waypoint' in coordinates:
            target_waypoint_loc = self.get_preparation_location(task['target_location'])
            command['target_waypoint_location'] = target_waypoint_loc
            command['target_waypoint_coords'] = coordinates['target_waypoint']
            self.get_logger().info(f"   ✅ Target Waypoint 포함: {target_waypoint_loc}")
        
        # JSON으로 변환
        command_json = json.dumps(command)
        msg = String()
        msg.data = command_json
        
        # 해당 로봇에게 발행
        if assigned_robot == 'robot1':
            self.robot1_pub.publish(msg)
            self.get_logger().info(f"✅ robot1에게 Task 전송: {task['task_type']}")
        elif assigned_robot == 'robot5':
            self.robot5_pub.publish(msg)
            self.get_logger().info(f"✅ robot5에게 Task 전송: {task['task_type']}")
    
    def send_double_exit_task(self, task: Dict, coordinates: Dict):
        """
        EXIT_DOUBLE Task 전송 (하나의 명령으로)
        
        로봇 노드에서 blocking → temp, start → target 순차 처리
        """
        assigned_robot = task['assigned_robot']
        
        # EXIT_DOUBLE 명령 (모든 정보 포함)
        command = {
            'task_id': task['task_id'],
            'task_type': 'EXIT_DOUBLE',
            'vehicle_plate': task['vehicle_plate'],  # 메인 차량
            'blocking_vehicle': task.get('blocking_vehicle'),  # 방해 차량
            
            # 메인 차량 경로
            'start_location': task['start_location'],
            'start_coords': coordinates['start'],
            'target_location': task['target_location'],
            'target_coords': coordinates['target'],
            
            # 방해 차량 경로
            'blocking_location': task.get('blocking_location'),
            'blocking_coords': coordinates.get('blocking'),
            'temp_location': task.get('temp_location'),
            'temp_coords': coordinates.get('temp'),
            
            'priority': task['priority']
        }
        
        # ⭐ Start Waypoint 추가 (메인 차량)
        if 'start_waypoint' in coordinates:
            start_waypoint_loc = self.get_preparation_location(task['start_location'])
            command['start_waypoint_location'] = start_waypoint_loc
            command['start_waypoint_coords'] = coordinates['start_waypoint']
            self.get_logger().info(f"   ✅ Start Waypoint 포함: {start_waypoint_loc}")
        
        # ⭐ Target Waypoint 추가 (메인 차량)
        if 'target_waypoint' in coordinates:
            target_waypoint_loc = self.get_preparation_location(task['target_location'])
            command['target_waypoint_location'] = target_waypoint_loc
            command['target_waypoint_coords'] = coordinates['target_waypoint']
            self.get_logger().info(f"   ✅ Target Waypoint 포함: {target_waypoint_loc}")
        
        # ⭐ Blocking Waypoint 추가 (방해 차량)
        if 'blocking_waypoint' in coordinates:
            blocking_loc = task.get('blocking_location')
            waypoint_loc = self.get_preparation_location(blocking_loc)
            command['blocking_waypoint_location'] = waypoint_loc
            command['blocking_waypoint_coords'] = coordinates['blocking_waypoint']
            self.get_logger().info(f"   ✅ Blocking Waypoint 포함: {waypoint_loc}")
        
        # JSON으로 변환
        command_json = json.dumps(command)
        msg = String()
        msg.data = command_json
        
        # 할당된 로봇에게 발행
        if assigned_robot == 'robot1':
            self.robot1_pub.publish(msg)
            self.get_logger().info(f"✅ robot1에게 EXIT_DOUBLE Task 전송")
        elif assigned_robot == 'robot5':
            self.robot5_pub.publish(msg)
            self.get_logger().info(f"✅ robot5에게 EXIT_DOUBLE Task 전송")
        
        self.get_logger().info(f"   📦 포함 정보:")
        self.get_logger().info(f"      - 메인 차량: {task['vehicle_plate']}")
        self.get_logger().info(f"      - 방해 차량: {task.get('blocking_vehicle')}")
        self.get_logger().info(f"      - 메인 경로: {task['start_location']} → {task['target_location']}")
        self.get_logger().info(f"      - 방해 경로: {task.get('blocking_location')} → {task.get('temp_location')}")



def main(args=None):
    rclpy.init(args=args)
    
    print("\n" + "=" * 60)
    print("🤖 Robot Manager 시작 (Realtime 버전)")
    print("=" * 60)
    print("\n📋 기능:")
    print("   - tasks 테이블 Realtime 구독 ✅")
    print("   - DB 폴링 제거 (부하 최소화) ✅")
    print("   - location_id → 좌표 변환 (캐싱)")
    print("   - Start/Target Waypoint 자동 추가 ⭐")
    print("   - Task 명령 발행:")
    print("     * /task_command/robot1")
    print("     * /task_command/robot5")
    print("\n💡 로봇 노드를 실행해서 테스트하세요!")
    print("=" * 60 + "\n")
    
    try:
        manager = RobotManager()
        rclpy.spin(manager)
    
    except KeyboardInterrupt:
        print("\n\n👋 종료")
    except Exception as e:
        print(f"\n❌ 에러: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'manager' in locals():
            manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()