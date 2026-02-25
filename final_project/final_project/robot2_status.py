import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from supabase import create_client, Client
import os
from datetime import datetime, timedelta

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        
        # 로봇 이름 파라미터
        self.declare_parameter('robot_name', 'robot5')
        self.robot_name = self.get_parameter('robot_name').value
        
        # ✅ Supabase 파라미터 선언 및 가져오기
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')  # 전체 키 입력
        
        # ✅ 파라미터 값 가져오기 (환경 변수 아님!)
        supabase_url = self.get_parameter('supabase_url').value
        supabase_key = self.get_parameter('supabase_key').value
        
        # ✅ 검증
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
        
        # 마지막 업데이트 시간
        self.last_update_time = datetime.now()
        self.update_interval = timedelta(minutes=10)
        
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
        
        # 주기적 업데이트 타이머
        self.timer = self.create_timer(60.0, self.periodic_update_check)
        
        self.get_logger().info(f'Robot status publisher for {self.robot_name} started')

    def battery_callback(self, msg: BatteryState):
        """배터리 상태 업데이트"""
        changed = False
        
        new_percentage = msg.percentage * 100 if msg.percentage else None
        if self.current_status['battery_percentage'] != new_percentage:
            self.current_status['battery_percentage'] = new_percentage
            changed = True
        
        if self.current_status['battery_voltage'] != msg.voltage:
            self.current_status['battery_voltage'] = msg.voltage
            changed = True
        
        is_charging = msg.current > 0 if msg.current else False
        if self.current_status['is_charging'] != is_charging:
            self.current_status['is_charging'] = is_charging
            changed = True
        
        if changed:
            self.update_database()

    def dock_callback(self, msg: Bool):
        """도킹 상태 업데이트"""
        if self.current_status['is_docked'] != msg.data:
            self.current_status['is_docked'] = msg.data
            self.update_database()

    def pose_callback(self, msg: PoseWithCovarianceStamped):


        """위치 정보 업데이트"""
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
        
        if changed and (datetime.now() - self.last_update_time > timedelta(minutes=1)):
            self.update_database()

    def busy_callback(self, msg: Bool):
        """작업 중 상태 업데이트"""
        if self.current_status['is_busy'] != msg.data:
            self.current_status['is_busy'] = msg.data
            self.update_database()

    def periodic_update_check(self):
        """주기적으로 업데이트가 필요한지 체크"""
        time_since_update = datetime.now() - self.last_update_time
        
        if time_since_update >= self.update_interval:
            self.get_logger().info('Periodic update triggered (10 min interval)')
            self.update_database(force=True)

    def update_database(self, force=False):
        """데이터베이스 업데이트"""
        try:
            update_data = {
                **self.current_status,
                'last_updated': datetime.now().isoformat()
            }
            
            result = self.supabase.table('robot_status').upsert(
                update_data,
                on_conflict='robot_name'
            ).execute()
            
            self.last_update_time = datetime.now()
            
            if force:
                self.get_logger().info(f'Database updated (forced): {self.robot_name}')
            else:
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