#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from parking_msgs.msg import SlotStates, SlotState
from std_msgs.msg import Header
from geometry_msgs.msg import Pose


class ParkingSlotManager(Node):
    def __init__(self):
        super().__init__('parking_slot_manager')

        self.declare_parameter('num_slots', 6)
        self.declare_parameter('occupied_slots', [1, 2])

        self.num_slots = self.get_parameter('num_slots').value
        self.occupied_slots = set(self.get_parameter('occupied_slots').value)

        self.pub = self.create_publisher(SlotStates, 'slot_states', 10)
        self.timer = self.create_timer(0.5, self.publish_slot_states)

        self.get_logger().info(
            f'ParkingSlotManager started. num_slots={self.num_slots}, '
            f'occupied_slots={sorted(self.occupied_slots)}'
        )

    def publish_slot_states(self):
        msg = SlotStates()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        slots = []
        for slot_id in range(1, self.num_slots + 1):
            s = SlotState()
            s.slot_id = slot_id
            s.occupied = slot_id in self.occupied_slots
            s.car_id = slot_id if s.occupied else 0
            s.pose = Pose()
            slots.append(s)

        msg.slots = slots
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSlotManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
