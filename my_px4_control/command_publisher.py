import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_commander')
        self.publisher_ = self.create_publisher(String, '/my_px4_control/vehicle_cmd', 10)
        
        # 指令映射表
        self.cmd_map = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'space': 'stop',
            'l': 'land',
            'q': 'quit'
        }
        self.get_logger().info("指揮官節點已啟動。輸入 w, a, s, d, l, space 或 q 控制。")

    def send_command(self, cmd_text):
        msg = String()
        msg.data = cmd_text
        self.publisher_.publish(msg)
        self.get_logger().info(f"已發送指令: '{cmd_text}'")

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()

    try:
        while rclpy.ok():
            # 使用 input() 讀取鍵盤輸入
            user_input = input("請輸入指令 (w:前, s:後, a:左, d:右, space:停, q:退出): ").lower()

            if user_input == 'w':
                node.send_command('forward')
            elif user_input == 's':
                node.send_command('backward')
            elif user_input == 'a':
                node.send_command('left')
            elif user_input == 'd':
                node.send_command('right')
            elif user_input == ' ' or user_input == 'space':
                node.send_command('stop')
            elif user_input == 'l':
                node.send_command('land')
            elif user_input == 'q':
                print("正在退出...")
                break
            else:
                print("無效指令，請重新輸入。")

            # 依你要求使用 spin_once 處理後台任務（雖然此例主要為發送，但這是好習慣）
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()