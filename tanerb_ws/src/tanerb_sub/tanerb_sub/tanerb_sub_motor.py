import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tanerb_sub.DM_lib.damiao_motor import Motor, MotorControl, DM_Motor_Type, Control_Type
import time
import subprocess
#!/usr/bin/env python3

class MotorJointStateSubscriber(Node):
    def __init__(self):
        super().__init__('motor_joint_state_subscriber')
        
        # Configuration for 7 motors
        self.CAN_INTERFACE = "can0"
        self.CAN_BITRATE = 1000000

        # Bring up CAN interface
        try:
            subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '1000000'], check=True)
            self.get_logger().info('CAN interface can0 brought up successfully')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to bring up CAN interface: {e}')
        
        
        # Define motor configurations with joint names
        self.motor_configs = [
            {'can_id': 0x01, 'master_id': 0x11, 'type': DM_Motor_Type.DM4340_48V, 'joint_name': 'joint1'},
            {'can_id': 0x02, 'master_id': 0x12, 'type': DM_Motor_Type.DM4340_48V, 'joint_name': 'joint2'},
            {'can_id': 0x03, 'master_id': 0x13, 'type': DM_Motor_Type.DM4340_48V, 'joint_name': 'joint3'},
            {'can_id': 0x04, 'master_id': 0x14, 'type': DM_Motor_Type.DM4340_48V, 'joint_name': 'joint4'},
            {'can_id': 0x05, 'master_id': 0x15, 'type': DM_Motor_Type.DM4310_48V, 'joint_name': 'joint5'},
            {'can_id': 0x06, 'master_id': 0x16, 'type': DM_Motor_Type.DM4310_48V, 'joint_name': 'joint6'},
            {'can_id': 0x07, 'master_id': 0x17, 'type': DM_Motor_Type.DM4310_48V, 'joint_name': 'joint7'},
        ]
        
        # Joint name to motor index mapping
        self.joint_to_motor_idx = {}
        
        # Initialize motor control
        self.mc = MotorControl(channel=self.CAN_INTERFACE, bitrate=self.CAN_BITRATE)
        self.motors = []
        
        # Create and add motors
        for idx, config in enumerate(self.motor_configs):
            motor = Motor(
                MotorType=config['type'],
                SlaveID=config['can_id'],
                MasterID=config['master_id']
            )
            self.mc.addMotor(motor)
            self.motors.append(motor)
            self.joint_to_motor_idx[config['joint_name']] = idx
        
        # Enable all motors
        time.sleep(1.0)  # Wait before enabling
        for motor in self.motors:
            self.mc.enable(motor)
            time.sleep(0.1)
        
        # Switch all motors to POS_VEL control mode
        self.get_logger().info('Switching motors to POS_VEL mode...')
        for i, motor in enumerate(self.motors):
            success = self.mc.switchControlMode(motor, Control_Type.POS_VEL)
            if success:
                self.get_logger().info(f'Motor {i+1} (ID: {motor.SlaveID}) switched to POS_VEL mode')
            else:
                self.get_logger().warn(f'Motor {i+1} (ID: {motor.SlaveID}) failed to switch to POS_VEL mode')
            # time.sleep(0.1)
        
        self.get_logger().info(f'Enabled {len(self.motors)} motors')
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Motor Joint State Subscriber started')
    
    def joint_state_callback(self, msg):
        # Map joint names to positions
        joint_values = []
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_to_motor_idx:
                motor_idx = self.joint_to_motor_idx[joint_name]
                motor = self.motors[motor_idx]
                
                target_pos = msg.position[i]
                target_vel = 5.0    # msg.velocity[i] if i < len(msg.velocity) else 0.0
                
                joint_values.append(f"{joint_name}: {target_pos:.4f} rad")
                
                # Send control command
                self.mc.control_Pos_Vel(
                    motor,
                    P_desired=target_pos,
                    V_desired=target_vel
                )
        
        # Print all joint values
        if joint_values:
            self.get_logger().info(f"Joint positions: {', '.join(joint_values)}")
        
        # Receive feedback
        self.mc.recv()
    
    def shutdown(self):
        """Disable all motors on shutdown"""
        self.get_logger().info('Shutting down motors...')
        for motor in self.motors:
            self.mc.disable(motor)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MotorJointStateSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()