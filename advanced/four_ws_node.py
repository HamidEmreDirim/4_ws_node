#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from geometry import Point
from drive_module import DriveModule
from steering_controller import FourWSController
from control import BodyMotionCommand


class FourWSNode(Node):
    def __init__(self):
        super().__init__('four_ws_node')

        # --- robot geometrisi ------------------------------------------------
        L, W = 1.48996, 1.2405
        self.mods = []
        for name, sx, sy in [('left_front',  0.5,  0.5),
                             ('left_rear',  -0.5,  0.5),
                             ('right_rear', -0.5, -0.5),
                             ('right_front', 0.5, -0.5)]:
            self.mods.append(DriveModule(name, f'{name}_steer', f'{name}_drive',
                                         Point(sx*L, sy*W), 0.04, 0.05, v_max=6.0))

        self.ctrl = FourWSController(self.mods, self.get_logger().info)

        self.sub = self.create_subscription(Joy, 'joy', self.cb_joy, 10)
        self.create_timer(0.02, self.cb_timer)  # 50 Hz

    # ------------------------------------------------------------------------
    def cb_joy(self, j: Joy):
        vx =  7.5 * j.axes[1]
        vy =  7.5 * j.axes[0]
        wz = 10.0 * j.axes[3]
        self.ctrl.set_command(BodyMotionCommand(1.0, vx, vy, wz))

    def cb_timer(self):
        self.ctrl.periodic()


def main():
    rclpy.init()
    node = FourWSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
