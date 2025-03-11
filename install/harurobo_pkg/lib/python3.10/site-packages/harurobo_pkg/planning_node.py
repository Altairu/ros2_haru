import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('planning_node')  # ノード名を変更
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd_vel', 10)

        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回
        self.action_number = 0  # 指示番号の初期値

    def listener_callback(self, msg):
        data = msg.data.split(',')
        vx = float(data[0])
        vy = float(data[1])
        omega = float(data[2])
        emergency_stop = int(data[5])

        self.get_logger().info(f"Received data: {data}")

        # 非常停止処理
        if emergency_stop == 1:
            self.send_velocity_command(0.0, 0.0, 0.0, float(255))
            return

        # コントローラーの操作に基づいて指示番号を変更
        if data[10] == 'circle':
            self.action_number = min(self.action_number + 1, 10)
        elif data[10] == 'square':
            self.action_number = max(self.action_number - 1, 0)

        self.send_velocity_command(vx, vy, omega, float(self.action_number))

    def send_velocity_command(self, vx, vy, omega, action_number):
        msg = Float32MultiArray()
        msg.data = [vx, vy, omega, action_number]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent velocity command: {msg.data}")

    def timer_callback(self):
        pass  # タイマーコールバックの追加

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()