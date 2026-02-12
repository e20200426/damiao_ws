import time
import rclpy
from rclpy.node import Node
from tanerb_sub.DM_lib.damiao_motor import Motor, MotorControl, DM_Motor_Type
from sensor_msgs.msg import JointState

# -------------------------
# Configuration
# -------------------------
CAN_INTERFACE = "can0"
CAN_BITRATE = 1000000
MOTOR_TYPE = DM_Motor_Type.DM4310_48V
SLAVE_ID = 0x06
MASTER_ID = 0x16
JOINT_NAME = "motor_1"  # Change this to a suitable name for your robot's joint

class MotorPositionNode(Node):
    def __init__(self):
        super().__init__('read_pos_dm4310')
        
        # Initialize motor control and motor objects
        self.mc = MotorControl(channel=CAN_INTERFACE, bitrate=CAN_BITRATE)
        self.motor = Motor(MotorType=MOTOR_TYPE, SlaveID=SLAVE_ID, MasterID=MASTER_ID)
        self.mc.addMotor(self.motor)

        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Start a timer to read the motor position every 50ms
        self.timer = self.create_timer(0.05, self.read_motor_position)

        self.get_logger().info("📡 MIT mode: reading position")

    def read_motor_position(self):
        try:
            # Send ZERO-force MIT frame → triggers feedback
            self.mc.controlMIT(
                self.motor,
                kp=0.0,
                kd=0.0,
                q=0.0,
                dq=0.0,
                tau=0.0
            )

            # Receive the latest feedback from the motor
            self.mc.recv()

            # Create a JointState message
            joint_state = JointState()
            joint_state.name = [JOINT_NAME]
            
            # Ensure motor.getPosition() returns a float and assign to position as a list of floats
            joint_state.position = [float(self.motor.getPosition())]  # Position as a list of float
            joint_state.velocity = [float(self.motor.getVelocity())]  # Velocity as a list of float
            joint_state.effort = [float(self.motor.getTorque())]      # Torque as a list of float

            # Publish the joint state
            self.joint_state_publisher.publish(joint_state)

            # Log the motor feedback
            self.get_logger().info(
                f"Pos: {self.motor.getPosition():+.6f} rad | "
                f"Vel: {self.motor.getVelocity():+.6f} rad/s | "
                f"Tau: {self.motor.getTorque():+.6f} Nm"
            )

        except Exception as e:
            self.get_logger().error(f"Error reading motor data: {e}")


def main(args=None):
    rclpy.init(args=args)

    motor_position_node = MotorPositionNode()

    try:
        rclpy.spin(motor_position_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_position_node.mc.disable(motor_position_node.motor)
        rclpy.shutdown()


if __name__ == '__main__':
    main()