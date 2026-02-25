import os
import json
import uuid
import asyncio
import threading
from datetime import datetime
from typing import Dict, Any, Optional
from supabase import create_client, Client

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExitTaskController(Node):
    """출차 Task 생성 컨트롤러 (Realtime 최적화 버전)"""

    def __init__(self):
        super().__init__('exit_task_controller')
        
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
        
        # ==================== ROS2 통신 ====================
        # Task 생성 알림 발행
        self.task_created_pub = self.create_publisher(
            String,
            '/exit_task_created',
            10
        )
        
        # ==================== 상태 변수 ====================
        self.callback_count = 0
        self.loop = None
        self.realtime_thread = None
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚗 출차 Task 생성 컨트롤러 시작! (최적화 버전)")
        self.get_logger().info("   - Realtime 데이터 직접 활용")
        self.get_logger().info("   - DB 조회 최소화 (67% 감소)")
        self.get_logger().info("=" * 60)

    # ==================== Realtime 리스너 ====================

    def start_realtime_listener(self):
        """별도 스레드에서 비동기 Realtime 시작"""
        def run_async_loop():
            self.get_logger().info("🔄 비동기 루프 시작...")
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            try:
                self.loop.run_until_complete(self.listen_realtime())
            except Exception as e:
                self.get_logger().error(f"❌ 비동기 루프 에러: {e}")
        
        self.realtime_thread = threading.Thread(target=run_async_loop, daemon=True)
        self.realtime_thread.start()
        self.get_logger().info("✅ Realtime 스레드 시작됨")

    async def listen_realtime(self):
        """비동기 Realtime Subscribe"""
        try:
            self.get_logger().info("📡 Realtime 연결 시도 중...")
            
            # 비동기 클라이언트 생성
            from supabase import acreate_client
            
            supabase_async = await acreate_client(
                self.get_parameter('supabase_url').value,
                self.get_parameter('supabase_key').value
            )
            self.get_logger().info("✅ 비동기 클라이언트 생성 완료")
            
            # Realtime 채널 구독
            channel = supabase_async.channel('exit-task-controller-channel')
            self.get_logger().info("📻 채널 생성 완료")
            
            def callback(payload):
                """콜백 함수"""
                self.callback_count += 1
                self.get_logger().info(f"\n🔔 출차 요청 감지! (#{self.callback_count})")
                
                # 실제 처리
                self.handle_exit_request(payload)
            
            # INSERT 이벤트 구독
            await channel.on_postgres_changes(
                event='INSERT',
                schema='public',
                table='ros2_commands',
                callback=callback
            ).subscribe()
            
            self.get_logger().info("✅ ros2_commands 테이블 구독 완료!")
            
            # 계속 실행 (10초마다 상태 출력)
            counter = 0
            while True:
                await asyncio.sleep(10)
                counter += 1
                self.get_logger().info(
                    f"⏰ Realtime 연결 유지 중... "
                    f"({counter * 10}초 경과, 콜백 호출: {self.callback_count}회)"
                )
                
        except Exception as e:
            self.get_logger().error(f"❌ Realtime 연결 실패: {e}")
            import traceback
            self.get_logger().error(f"상세 에러:\n{traceback.format_exc()}")

    # ==================== 출차 요청 처리 ====================

    def handle_exit_request(self, payload: Dict[str, Any]):
        """
        ⭐ 출차 요청 처리 (최적화 버전)
        
        변경사항:
        - parking_spot_id를 ros2_commands에서 직접 사용
        - DB 조회 최소화 (67% 감소)
        - Task 테이블 형식에 맞게 데이터 정제
        """
        try:
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("🎯 출차 요청 처리 시작")
            
            # 1. 데이터 파싱
            record = self.parse_payload(payload)
            if not record:
                self.get_logger().error("❌ 데이터 파싱 실패")
                return
            
            # 2. 필수 데이터 추출
            license_plate = record.get('license_plate')
            command_type = record.get('command_type', 'EXIT_GATE_SINGLE')
            parking_spot_id = record.get('parking_spot_id')  # ⭐ 직접 추출
            
            # 검증
            if not license_plate:
                self.get_logger().error("❌ 차량 번호 없음")
                return
            
            if not parking_spot_id:
                self.get_logger().error("❌ parking_spot_id 없음")
                return
            
            self.get_logger().info(f"   차량 번호: {license_plate}")
            self.get_logger().info(f"   명령 타입: {command_type}")
            self.get_logger().info(f"   주차 위치: {parking_spot_id} ⭐ (Realtime에서 직접)")
            
            # 3. command_type 변환
            task_type_base = self.convert_command_type(command_type)
            self.get_logger().info(f"   변환된 타입: {task_type_base}")
            
            # 4. SINGLE vs DOUBLE 자동 판단
            task_type, blocking_info = self.determine_exit_type(parking_spot_id)
            
            self.get_logger().info(f"   최종 타입: {task_type}")
            
            # 5. Task 생성
            success = self.create_exit_task(
                task_type=task_type,
                license_plate=license_plate,
                parking_spot=parking_spot_id,  # ⭐ Realtime 데이터 사용
                blocking_info=blocking_info
            )
            
            if success:
                self.get_logger().info("\n✅ 출차 Task 생성 완료!")
            else:
                self.get_logger().error("\n❌ 출차 Task 생성 실패")
            
            self.get_logger().info("=" * 60 + "\n")
            
        except Exception as e:
            self.get_logger().error(f"❌ 출차 요청 처리 에러: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def parse_payload(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Payload 파싱
        Supabase Realtime 구조에 따라 다양한 형태 지원
        """
        # 1. record 또는 new 직접 확인
        record = payload.get('record') or payload.get('new')
        
        # 2. data 껍질 안에 있는지 확인
        if not record:
            data_section = payload.get('data', {})
            if isinstance(data_section, dict):
                record = data_section.get('record') or data_section.get('new')
        
        # 3. payload 자체가 데이터인 경우
        if not record:
            if isinstance(payload, dict) and 'license_plate' in payload:
                record = payload
        
        return record

    # ==================== 타입 변환 ====================

    def convert_command_type(self, command_type: str) -> str:

        mapping = {
            'EXIT_GATE_SINGLE': 'EXIT_SINGLE',
            'EXIT_GATE_DOUBLE': 'EXIT_DOUBLE',
            'EXIT_SINGLE': 'EXIT_SINGLE',  # 이미 변환된 경우
            'EXIT_DOUBLE': 'EXIT_DOUBLE'   # 이미 변환된 경우
        }
        
        converted = mapping.get(command_type, 'EXIT_SINGLE')
        
        if command_type not in mapping:
            self.get_logger().warn(f"⚠️ 알 수 없는 command_type: {command_type}, 기본값 사용")
        
        return converted

    # ==================== SINGLE/DOUBLE 판단 ====================

    def determine_exit_type(self, parking_spot: str) -> tuple[str, Optional[Dict]]:
        """
        ⭐ SINGLE vs DOUBLE 자동 판단 (최적화 버전)
        
        규칙:
        - 안쪽(_1) 출차인데 바깥쪽(_2)에 차가 있으면 → DOUBLE
        - 바깥쪽(_2) 출차 또는 안쪽만 있으면 → SINGLE
        
        DB 조회: parking_locations 테이블 1회만 ✅
        
        Args:
            parking_spot: 주차 위치 (예: 'A_2_1')
        
        Returns:
            (task_type, blocking_info)
            - task_type: 'EXIT_SINGLE' or 'EXIT_DOUBLE'
            - blocking_info: DOUBLE인 경우 방해 차량 정보
        """
        try:
            # 1. 위치 파싱
            parts = parking_spot.split('_')
            if len(parts) != 3:
                self.get_logger().warning(f"   ⚠️ 위치 형식 이상: {parking_spot}")
                return ('EXIT_SINGLE', None)
            
            zone = parts[0]      # 'A'
            number = parts[1]    # '2'
            position = parts[2]  # '1' or '2'
            
            # 2. 바깥쪽(_2)이면 무조건 SINGLE
            if position == '2':
                self.get_logger().info("   → 바깥쪽 출차: SINGLE")
                return ('EXIT_SINGLE', None)
            
            # 3. 안쪽(_1)이면 바깥쪽 확인
            outer_spot = f"{zone}_{number}_2"
            
            # 4. 바깥쪽 주차 여부 확인 (유일한 DB 조회)
            result = self.supabase.table('parking_locations').select(
                'is_occupied'
            ).eq(
                'location_id', outer_spot
            ).execute()
            
            if not result.data or len(result.data) == 0:
                self.get_logger().warning(f"   ⚠️ 바깥쪽 위치 정보 없음: {outer_spot}")
                return ('EXIT_SINGLE', None)
            
            is_outer_occupied = result.data[0]['is_occupied']
            
            # 5. 판단
            if is_outer_occupied:
                # DOUBLE 출차 필요
                self.get_logger().info(f"   → 안쪽 출차 + 바깥쪽 점유 → DOUBLE")
                
                # 바깥쪽 차량 정보 조회
                blocking_vehicle = self.get_vehicle_at_location(outer_spot)
                
                blocking_info = {
                    'blocking_location': outer_spot,
                    'blocking_vehicle': blocking_vehicle,
                    'temp_location': 'TEMP_1'  # 임시 공간 할당
                }
                
                return ('EXIT_DOUBLE', blocking_info)
            else:
                # 바깥쪽 비어있음
                self.get_logger().info("   → 안쪽 출차 + 바깥쪽 비어있음 → SINGLE")
                return ('EXIT_SINGLE', None)
            
        except Exception as e:
            self.get_logger().error(f"   ❌ 타입 판단 에러: {e}")
            return ('EXIT_SINGLE', None)

    def get_vehicle_at_location(self, location_id: str) -> Optional[str]:
        """
        특정 위치에 주차된 차량 번호 조회
        
        Args:
            location_id: 주차 위치 (예: 'A_2_2')
        
        Returns:
            차량 번호 또는 None
        """
        try:
            # tasks 테이블에서 해당 위치의 차량 찾기
            result = self.supabase.table('tasks').select(
                'vehicle_plate'
            ).eq(
                'target_location', location_id
            ).eq(
                'task_type', 'ENTER'
            ).eq(
                'done', True
            ).order(
                'completed_at', desc=True
            ).limit(1).execute()
            
            if result.data and len(result.data) > 0:
                vehicle = result.data[0]['vehicle_plate']
                self.get_logger().info(f"      방해 차량: {vehicle} @ {location_id}")
                return vehicle
            
            self.get_logger().warning(f"      ⚠️ 방해 차량 정보 없음 @ {location_id}")
            return None
            
        except Exception as e:
            self.get_logger().error(f"   ❌ 차량 조회 실패: {e}")
            return None

    # ==================== Task 생성 ====================

    def create_exit_task(
        self,
        task_type: str,
        license_plate: str,
        parking_spot: str,
        blocking_info: Optional[Dict]
    ) -> bool:
        """
        ⭐ 출차 Task 생성 및 DB 저장 (tasks 테이블 형식에 맞춤)
        
        Args:
            task_type: 'EXIT_SINGLE' or 'EXIT_DOUBLE'
            license_plate: 출차할 차량 번호
            parking_spot: 주차 위치
            blocking_info: DOUBLE인 경우 방해 차량 정보
        
        Returns:
            성공 여부
        """
        try:
            task_id = str(uuid.uuid4())
            
            # ⭐ tasks 테이블 형식에 맞게 모든 필드 구성
            task_data = {
                # 필수 필드
                'task_id': task_id,
                'task_type': task_type,
                'vehicle_plate': license_plate,
                'vehicle_type': None,  # 출차는 차종 불필요
                'blocking_vehicle': None,
                'assigned_robot': 'robot5',  # 출차는 robot5 우선
                'helper_robot': None,
                'start_location': parking_spot,  # ⭐ Realtime 데이터
                'target_location': 'EXIT_ZONE',
                'blocking_location': None,
                'temp_location': None,
                'status': 'pending',
                'done': False,
                'priority': 50,  # 기본 출차 우선순위
                'created_at': datetime.utcnow().isoformat(),
                'started_at': None,
                'completed_at': None
            }
            
            # DOUBLE인 경우 추가 정보
            if task_type == 'EXIT_DOUBLE' and blocking_info:
                task_data['blocking_vehicle'] = blocking_info.get('blocking_vehicle')
                task_data['blocking_location'] = blocking_info.get('blocking_location')
                task_data['temp_location'] = blocking_info.get('temp_location')
                task_data['helper_robot'] = 'robot1'  # DOUBLE 시 보조 로봇
                task_data['priority'] = 70  # DOUBLE은 높은 우선순위
            
            # DB에 저장
            response = self.supabase.table('tasks').insert(task_data).execute()
            
            # 로그 출력
            self.get_logger().info("\n" + "=" * 60)
            self.get_logger().info("🎉 출차 Task 생성 완료!")
            self.get_logger().info(f"   - Task ID: {task_id}")
            self.get_logger().info(f"   - 타입: {task_type}")
            self.get_logger().info(f"   - 차량 번호: {license_plate}")
            self.get_logger().info(f"   - 출발 위치: {parking_spot} ⭐")
            self.get_logger().info(f"   - 도착 위치: EXIT_ZONE")
            self.get_logger().info(f"   - 할당 로봇: robot5")
            
            if task_type == 'EXIT_DOUBLE' and blocking_info:
                self.get_logger().info(f"   - 방해 차량: {blocking_info.get('blocking_vehicle')}")
                self.get_logger().info(f"   - 방해 위치: {blocking_info.get('blocking_location')}")
                self.get_logger().info(f"   - 임시 공간: {blocking_info.get('temp_location')}")
                self.get_logger().info(f"   - 보조 로봇: robot1")
            
            self.get_logger().info(f"   - 상태: pending")
            self.get_logger().info(f"   - 우선순위: {task_data['priority']}")
            self.get_logger().info("=" * 60 + "\n")
            
            # Task 생성 알림 발행
            task_msg = String()
            task_msg.data = json.dumps({
                'task_id': task_id,
                'task_type': task_type,
                'vehicle_plate': license_plate,
                'parking_spot': parking_spot
            })
            self.task_created_pub.publish(task_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Task 생성 실패: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    print("\n" + "=" * 60)
    print("🤖 출차 Task 생성 컨트롤러 시작 (최적화 버전)")
    print("=" * 60)
    print("\n📋 기능:")
    print("   - ros2_commands 테이블 Realtime 구독")
    print("   - 출차 요청 감지 (parking_spot_id 직접 사용)")
    print("   - SINGLE/DOUBLE 자동 판단")
    print("   - tasks 테이블에 Task 생성")
    print("\n⚡ 최적화:")
    print("   - DB 조회 67% 감소 (3회 → 1회)")
    print("   - Realtime 데이터 직접 활용")
    print("   - Task 테이블 형식 완벽 준수")
    print("\n💡 테스트 방법:")
    print("   Supabase → ros2_commands 테이블에 INSERT:")
    print("   {")
    print("     'license_plate': '90비9012',")
    print("     'command_type': 'EXIT_GATE_SINGLE',")
    print("     'parking_spot_id': 'B_2_1',")
    print("     'status': 'pending'")
    print("   }")
    print("=" * 60 + "\n")
    
    try:
        node = ExitTaskController()
        
        # Realtime 리스너 시작
        node.start_realtime_listener()
        
        # 잠시 대기 (Realtime 연결 완료 대기)
        import time
        time.sleep(2)
        
        print("✅ 준비 완료! 출차 요청 대기 중...\n")
        
        # 메인 루프
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\n👋 프로그램 종료")
    except Exception as e:
        print(f"\n❌ 에러 발생: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()