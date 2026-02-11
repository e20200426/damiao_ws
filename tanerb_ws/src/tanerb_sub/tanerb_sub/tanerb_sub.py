# joint_state_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Default joint state topic
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def joint_state_callback(self, msg):
        # Print the joint positions for all 7 joints
        self.get_logger().info(f'Joint Positions: {msg.position}')

        # Example of accessing individual joint positions:
        # Assuming you have 7 joints, you can access each position like this:
        if len(msg.position) >= 7:
            joint1_position = msg.position[0]
            joint2_position = msg.position[1]
            joint3_position = msg.position[2]
            joint4_position = msg.position[3]
            joint5_position = msg.position[4]
            joint6_position = msg.position[5]
            joint7_position = msg.position[6]
            self.get_logger().info(f'Joint 1: {joint1_position}, Joint 2: {joint2_position}, Joint 3: {joint3_position}, Joint 4: {joint4_position}, Joint 5: {joint5_position}, Joint 6: {joint6_position}, Joint 7: {joint7_position}')
        else:
            self.get_logger().warn('Insufficient number of joint positions received!')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
