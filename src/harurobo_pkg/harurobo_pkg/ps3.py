import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pygame
import os

class PS3ControllerNode(Node):
    def __init__(self):
        super().__init__('ps3_controller_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'send_can_message', 10)
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick initialized: {self.joystick.get_name()}")
        else:
            self.get_logger().error("No joystick found")
        self.action_number = 0  # 初期値を設定
        self.button_states = [False, False]  # 丸ボタンと四角ボタンの状態を保持
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回

    def timer_callback(self):
        pygame.event.pump()
        vx = self.joystick.get_axis(0) * 100.0  # 左ジョイスティックのx軸
        vy = self.joystick.get_axis(1) * 100.0*-1 #左ジョイスティックのy軸
        omega = self.joystick.get_axis(3) * 100.0  # 右ジョイスティックのx軸

        # 丸ボタン
        if self.joystick.get_button(1):
            if not self.button_states[0]:
                self.button_states[0] = True
        else:
            if self.button_states[0]:
                if self.action_number < 10:
                    self.action_number += 1
                self.button_states[0] = False

        # 四角ボタン
        if self.joystick.get_button(3):
            if not self.button_states[1]:
                self.button_states[1] = True
        else:
            if self.button_states[1]:
                if self.action_number > 0:
                    self.action_number -= 1
                self.button_states[1] = False

        data = Float32MultiArray()
        data.data = [float(vx), float(vy), float(omega), float(self.action_number)]
        self.publisher_.publish(data)

def main(args=None):
    rclpy.init(args=args)
    node = PS3ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
