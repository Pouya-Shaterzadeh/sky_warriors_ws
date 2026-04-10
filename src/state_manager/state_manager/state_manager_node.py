import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StateManagerNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.state = 'IDLE'
        self.create_subscription(String, '/qr_decoded', self.qr_callback, 10)
        self.create_subscription(String, '/manual_command', self.manual_callback, 10)
        self.create_subscription(String, '/uav_status', self.status_callback, 10)
        self.state_pub = self.create_publisher(String, '/swarm/state_change', 10)
        self.detach_pub = self.create_publisher(String, '/swarm/detach_command', 10)
        self.formation_pub = self.create_publisher(String, '/swarm/formation_command', 10)

    def qr_callback(self, msg):
        if 'detach' in msg.data:
            self.state = 'DETACHING'
            self.state_pub.publish(String(data='DETACHING'))
            # Example: extract UAV ID from QR and publish detach command
            self.detach_pub.publish(String(data='drone_2'))

    def manual_callback(self, msg):
        if msg.data == 'form_formation':
            self.state = 'FORMATION'
            self.state_pub.publish(String(data='FORMATION'))
            self.formation_pub.publish(String(data='form'))

    def status_callback(self, msg):
        # Extend with UAV status handling as needed
        pass

def main(args=None):
    rclpy.init(args=args)
    node = StateManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
