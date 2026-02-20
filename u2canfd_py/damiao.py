import sys
import time
import signal
import threading

from lib.DM_motor import (
    DM_Motor_Type,
    Control_Mode,
    Control_Mode_Code,
    DmActData,
    Motor_Control,
)

running = threading.Event()
running.set()

def signal_handler(signum, frame):
    running.clear()
    sys.stderr.write(f"\nInterrupt signal ({signum}) received.\n")
    sys.stderr.flush()

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    try:
        init_data1= []
        init_data2 = []
        canid1=0x01
        mstid1=0x11
        canid2=0x02
        mstid2=0x12
        canid3=0x03
        mstid3=0x13
        canid4=0x04
        mstid4=0x14
        canid5=0x05
        mstid5=0x15
        canid6=0x06
        mstid6=0x16
        canid7=0x07
        mstid7=0x17
        canid8=0x08
        mstid8=0x18
        canid9=0x09
        mstid9=0x19
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid1,
        #             mst_id=mstid1))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid2,
        #             mst_id=mstid2))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid3,
        #             mst_id=mstid3))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4340,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid4,
        #             mst_id=mstid4))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4340,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid5,
        #             mst_id=mstid5))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4340,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid6,
        #             mst_id=mstid6))
        init_data1.append(DmActData(
                    motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
                    mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
                    can_id=canid7,
                    mst_id=mstid7))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid8,
        #             mst_id=mstid8))
        # init_data1.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid9,
        #             mst_id=mstid9))
        # init_data2.append(DmActData(
        #             motorType=DM_Motor_Type.DM4310,  # 或者具体类型，如 DM_Motor_Type.DM4310
        #             mode=Control_Mode.POS_VEL_MODE,        # 如 Control_Mode.MIT_MODE
        #             can_id=canid2,
        #             mst_id=mstid2))

        #with Motor_Control(1000000, 5000000,"14AA044B241402B10DDBDAFE448040BB",init_data1) as control\
        #       ,Motor_Control(1000000, 5000000, "AA96DF2EC013B46B1BE4613798544085", init_data2) as control2:
        with Motor_Control(1000000, 1000000,"E067CA134F67746CCA5451F1BE23BAD8",init_data1) as control:
        #control=Motor_Control(1000000, 5000000,"14AA044B241402B10DDBDAFE448040BB",init_data1) 

            control.switchControlMode(control.getMotor(canid7), Control_Mode_Code.POS_VEL)

            while running.is_set():
                    desired_duration = 0.001  # 秒
                    current_time = time.perf_counter()
                    # kp: float, kd: float, q: float, dq: float, tau: float)
                    # control.control_mit(control.getMotor(canid1), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid2), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid3), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid4), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid5), 0.0, 0.0, 0.0, 0.0, 0.0)
                    #control.control_mit(control.getMotor(canid6), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid7), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid8), 0.0, 0.0, 0.0, 0.0, 0.0)
                    # control.control_mit(control.getMotor(canid9), 0.0, 0.0, 0.0, 0.0, 0.0)
                    
                    control.control_pos_vel(control.getMotor(canid7), 0.0, 2.0)
                    for id in range(1): 
                        pos = control.getMotor(canid7).Get_Position()
                        vel = control.getMotor(canid7).Get_Velocity()
                        tau = control.getMotor(canid7).Get_tau()
                        interval = control.getMotor(canid7).getTimeInterval()

                        print(f"canid is: {canid7} pos: {pos} vel: {vel} effort: {tau} time(s): {interval}", file=sys.stderr)

                    #control2.control_vel(control2.getMotor(canid2), -3.0)
                    #control.enable_all()
                    sleep_till = current_time + desired_duration
                    now = time.perf_counter()
                    if sleep_till > now:
                        time.sleep(sleep_till - now)
                    
            print("The program exited safely.") 
    except Exception as e:
        print(f"Error: hardware interface exception: {e}", file=sys.stderr)
    finally:
        #control.close()
        #control2.close()
        pass
