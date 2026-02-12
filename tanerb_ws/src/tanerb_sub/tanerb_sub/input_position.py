import time
from tanerb_ws.src.tanerb_sub.DM_lib.damiao_motor import Motor, MotorControl, DM_Motor_Type

# -------------------------
# Configuration
# -------------------------
CAN_INTERFACE = "can0"
CAN_BITRATE = 1000000          # Ensure this matches your setup
SLAVE_ID = 0x02
MASTER_ID = 0x12

# -------------------------
# Initialization
# -------------------------
mc = MotorControl(channel=CAN_INTERFACE, bitrate=CAN_BITRATE)

motor = Motor(
    MotorType=DM_Motor_Type.DM4340,
    SlaveID=SLAVE_ID,
    MasterID=MASTER_ID
)
mc.addMotor(motor)

mc.enable(motor)  # Enable the motor

# -------------------------
# Main Control Loop
# -------------------------
print("position control using mc.control_Pos_Vel()")
print("Type a target position in radians and press Enter.")
print("Ctrl+C to quit\n")

try:
    while True:
        # Get target position from user input
        target_pos = input("Target [rad]: ")
        if target_pos == "":
            continue  # Skip if input is empty
        
        target_pos = float(target_pos)  # Convert input to float

        # Get current position of the motor
        current_pos = motor.getPosition()

        # Calculate position error
        error = target_pos - current_pos

        # Call controlMIT to control the motor
        mc.control_Pos_Vel(
            motor,
            P_desired=target_pos,  # Use the correct parameter name for the position
            V_desired=5.0          # Use the correct parameter name for the velocity
        )


        # Display current motor position and error
        print(f"Current Pos: {current_pos:.4f} rad | Target Pos: {target_pos:.4f} rad | Error: {error:.4f} rad")

        time.sleep(0.02)  # Delay for control loop timing

except KeyboardInterrupt:
    print("Exiting position control using mc.control_Pos_Vel().")
    mc.disable(motor)  # Disable motor on exit
