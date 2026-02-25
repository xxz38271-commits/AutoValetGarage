#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from parking_msgs.msg import MissionArray
from supabase import create_client, Client



class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')

        self.sub = self.create_subscription(
            MissionArray,
            'raw_missions',
            self.raw_missions_callback,
            10
        )

        self.robot1_pub = self.create_publisher(
            MissionArray,
            'robot1/assigned_missions',
            10
        )
        self.robot5_pub = self.create_publisher(
            MissionArray,
            'robot5/assigned_missions',
            10
        )
        # Supabase 초기화
        SUPABASE_URL = "https://shmqecsymzygxatjsqid.supabase.co"
        SUPABASE_KEY = "sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_"

        self.supabase = create_client(SUPABASE_URL, SUPABASE_KEY)

        self.round_robin_toggle = 0

        self.get_logger().info('TaskAllocator started.')

    def raw_missions_callback(self, msg: MissionArray):
        robot1_list = []
        robot5_list = []

        for m in msg.missions:
            if m.mission_type in ['MOVE_FRONT_CAR', 'ALIGN_BASE_FRONT','BEEP_R1']:
                robot1_list.append(m)
            elif m.mission_type in ['EXIT_TARGET_CAR', 'ALIGN_BASE_EXIT','BEEP_R5']:
                robot5_list.append(m)
            else:
                if self.round_robin_toggle == 0:
                    robot1_list.append(m)
                    self.round_robin_toggle = 1
                else:
                    robot5_list.append(m)
                    self.round_robin_toggle = 0

        task_id_for_db = None  # 🔹 어떤 로봇이든 처음으로 잡힌 task_id 저장용

        if robot1_list:
            out = MissionArray()
            out.missions = robot1_list
            self.robot1_pub.publish(out)
            self.get_logger().info(f'Assigned {len(robot1_list)} missions to robot1.')

            # robot1 리스트에서 task_id 하나만 후보로 저장
            if hasattr(robot1_list[0], 'command_id'):
                task_id_for_db = robot1_list[0].command_id

        if robot5_list:
            out = MissionArray()
            out.missions = robot5_list
            self.robot5_pub.publish(out)
            self.get_logger().info(f'Assigned {len(robot5_list)} missions to robot5.')

            # 아직 task_id를 못 정했고, robot5에 command_id가 있으면 그걸 사용
            if task_id_for_db is None and hasattr(robot5_list[0], 'command_id'):
                task_id_for_db = robot5_list[0].command_id

        # 🔚 둘 중 하나라도 미션이 있었다면, 여기서 딱 한 번만 Supabase 업데이트
        if task_id_for_db:
            self.get_logger().info(f"[DB] updating task_id={task_id_for_db} → 'assigned'")
            self.mark_task_assigned(task_id_for_db)
        else:
            self.get_logger().warn("[DB] task_id_for_db 비어있음 → Supabase 업데이트 생략됨")

    # ========================================
    # 🔹 Task를 'assigned' 상태로 변경
    # ========================================
    def mark_task_assigned(self, task_id: str):
        try:
            self.supabase.table('tasks').update({
                'status': 'assigned'
            }).eq('task_id', task_id).execute()
            self.get_logger().info(f"✅ Task 할당됨: {task_id}")
        except Exception as e:
            self.get_logger().error(f"❌ Task 할당 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
