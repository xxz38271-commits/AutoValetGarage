import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from supabase import create_client, Client
from datetime import datetime, timedelta

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        
        # 로봇 이름 파라미터
        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').value
        
        # ✅ Supabase 파라미터 선언 및 가져오기
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')
        
        supabase_url = self.get_parameter('supabase_url').value
        supabase_key = self.get_parameter('supabase_key').value
        
        if not supabase_url or not supabase_key:
            self.get_logger().error('Supabase URL or Key is missing!')
            raise ValueError("Supabase configuration required")
        
        self.get_logger().info(f'Connecting to Supabase: {supabase_url}')
        self.supabase: Client = create_client(supabase_url, supabase_key)
        
        # 현재 상태 저장 (변경 감지용)
        self.current_status = {
            'robot_name': self.robot_name,
            'battery_percentage': None,
            'battery_voltage': None,
            'is_charging': None,
            'is_docked': None,
            'position_x': None,
            'position_y': None,
            'orientation_z': None,
            'orientation_w': None,
            'is_busy': None
        }
        
        # 마지막 전체 업데이트 시간 (10분 강제용)
        self.last_update_time = datetime.now()
        self.update_interval = timedelta(minutes=10)

        # ✅ 배터리용 별도 타임 스탬프 (3분 간격)
        self.battery_update_interval = timedelta(minutes=3)
        # 처음에는 바로 한 번 업데이트 될 수 있도록, 3분 전으로 세팅
        self.battery_last_update_time = datetime.now() - self.battery_update_interval
        
        # ROS2 구독자들
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.robot_name}/battery_state',
            self.battery_callback,
            10
        )
        
        self.dock_sub = self.create_subscription(
            Bool,
            f'{self.robot_name}/dock_status',
            self.dock_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.robot_name}/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.busy_sub = self.create_subscription(
            Bool,
            f'{self.robot_name}/is_busy',
            self.busy_callback,
            10
        )
        
        # 주기적 업데이트 타이머 (10초마다 체크만, 실제 강제는 10분 간격)
        self.timer = self.create_timer(10.0, self.periodic_update_check)
        
        self.get_logger().info(f'Robot status publisher for {self.robot_name} started')

    # -----------------------------
    # 콜백들
    # -----------------------------
    def battery_callback(self, msg: BatteryState):
        """배터리 상태 업데이트: DB는 3분에 한 번만"""
        changed = False
        
        new_percentage = msg.percentage * 100 if msg.percentage is not None else None
        if self.current_status['battery_percentage'] != new_percentage:
            self.current_status['battery_percentage'] = new_percentage
            changed = True
        
        if self.current_status['battery_voltage'] != msg.voltage:
            self.current_status['battery_voltage'] = msg.voltage
            changed = True
        
        is_charging = msg.current > 0 if msg.current is not None else False
        if self.current_status['is_charging'] != is_charging:
            self.current_status['is_charging'] = is_charging
            changed = True
        
        if changed:
            now = datetime.now()
            # ✅ 배터리는 3분마다만 DB 업데이트
            if now - self.battery_last_update_time >= self.battery_update_interval:
                self.get_logger().info("Battery changed → 3 min interval reached, updating DB")
                self.update_database()
                self.battery_last_update_time = now

    def dock_callback(self, msg: Bool):
        """도킹 상태 업데이트: 상태가 바뀌면 바로 DB 업데이트"""
        if self.current_status['is_docked'] != msg.data:
            self.current_status['is_docked'] = msg.data
            self.get_logger().info("Dock status changed → updating DB")
            self.update_database()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """위치 정보 업데이트: 변화가 있으면 1분 간격으로 업데이트"""
        changed = False
        
        new_x = round(msg.pose.pose.position.x, 3)
        new_y = round(msg.pose.pose.position.y, 3)
        new_z = round(msg.pose.pose.orientation.z, 3)
        new_w = round(msg.pose.pose.orientation.w, 3)
        
        if self.current_status['position_x'] != new_x:
            self.current_status['position_x'] = new_x
            changed = True
        
        if self.current_status['position_y'] != new_y:
            self.current_status['position_y'] = new_y
            changed = True
        
        if self.current_status['orientation_z'] != new_z:
            self.current_status['orientation_z'] = new_z
            changed = True
        
        if self.current_status['orientation_w'] != new_w:
            self.current_status['orientation_w'] = new_w
            changed = True
        
        # 기존 로직 유지: 너무 자주 안 올리도록 1분 간격
        if changed and (datetime.now() - self.last_update_time > timedelta(minutes=1)):
            self.get_logger().info("Pose changed → 1 min passed, updating DB")
            self.update_database()

    def busy_callback(self, msg: Bool):
        """작업 중 상태 업데이트: 상태가 바뀌면 바로 DB 업데이트"""
        if self.current_status['is_busy'] != msg.data:
            self.current_status['is_busy'] = msg.data
            self.get_logger().info("Busy status changed → updating DB")
            self.update_database()

    # -----------------------------
    # 타이머 콜백
    # -----------------------------
    def periodic_update_check(self):
        """주기적으로 (10분마다) 강제 업데이트"""
        time_since_update = datetime.now() - self.last_update_time
        
        if time_since_update >= self.update_interval:
            self.get_logger().info('Periodic forced update triggered (10 min interval)')
            self.update_database()

    # -----------------------------
    # DB 업데이트 함수
    # -----------------------------
    def update_database(self):
        """데이터베이스 업데이트"""
        try:
            update_data = {
                **self.current_status,
                'last_updated': datetime.now().isoformat()
            }
            
            self.supabase.table('robot_status').upsert(
                update_data,
                on_conflict='robot_name'
            ).execute()
            
            self.last_update_time = datetime.now()
            self.get_logger().info(f'Database updated: {self.robot_name}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to update database: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
