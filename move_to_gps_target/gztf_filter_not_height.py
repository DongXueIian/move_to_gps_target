import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TransformFilter(Node):
    def __init__(self):
        super().__init__('transform_filter')
        self.publisher = self.create_publisher(TFMessage, '/tf', 20)
        self.subscription = self.create_subscription(
            TFMessage,
            '/gz/tf',
            self.listener_callback,
            20)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: TFMessage):
        new_msg = TFMessage()
        for transform in msg.transforms:
            # Reset the translation to zero
            transform.transform.translation.z = 0.8
            # Keep the rotation as is
            new_msg.transforms.append(transform)
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TransformFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
