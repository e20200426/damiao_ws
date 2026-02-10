# Using USB-to-CANFD to Drive Damiao Motors — C++ Example

## Introduction
This is a C++ example for controlling Damiao motors.

The required hardware is **Damiao’s USB-to-CANFD** device.

The program has been tested on Ubuntu 20.04 and Ubuntu 22.04.

By default, the program:

Sets the motor with CAN ID = 0x01 and MST ID = 0x11 (DM4310 motor) to velocity mode
, Enables the motor, Makes the motor rotate, **Motor data bitrate is 5 Mbps**

***Note: When using 5 Mbps bitrate with multiple motors, a 120-ohm termination resistor must be connected to the last motor on the bus.***

## Software Architecture
Implemented in C++，ROS is NOT used

## Installation and Compilation
Install libusb：
```shell
sudo apt update
sudo apt install libusb-1.0-0-dev
```
Create workspace：
```shell
mkdir -p ~/catkin_ws
cd ~/catkin_ws
```
Copy the u2canfd folder from Gitee into ~/catkin_ws

Directory structure example:

<img src="./docs/srcCANFD.png" width="450" height="auto">

Build：
```shell
cd ~/catkin_ws/u2canfd
mkdir build
cd build
cmake ..
make
```
## Basic Usage
Use the latest Damiao PC tool to set the motor baud rate to 5 Mbps.

Set **USB-to-CANFD** permissions: 
Create a udev rule:
```shell
sudo nano /etc/udev/rules.d/99-usb.rules
```
Add：
```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"
```
Reload rules:
```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```
***Note: This only needs to be done once. No need to repeat after reboot or replug.***

Then, the program needs to find the Serial_Number of the **USB-to-CANFD**，Open a terminal in the build folder where you just compiled and run the dev_sn file:
```shell
cd ~/catkin_ws/u2canfd/build
./dev_sn
```
<img src="./docs/dev.png" width="700" height="auto">

The string of numbers following the SN in the image above is the Serial\_Number of the device.

Next, copy the Serial\_Number file, open main.cpp, and replace the Serial\_Number file in the program, as shown in the image below:

<img src="./docs/motor_control.png" width="850" height="auto">

Then recompile, open the terminal and type:
```shell
cd ~/catkin_ws/u2canfd/build
make
```

Open a terminal in the build folder where you just compiled and run the dm_main file:
```shell
cd ~/catkin_ws/u2canfd/build
./dm_main
```
At this point, you will see the motor light turn green and start rotating.

## Advanced Usage
Below is a step-by-step guide on how to use this program to simultaneously control nine DM4310 motors at a baud rate of 5 MHz and a kHz frequency.

***Note: At a 5 MHz baud rate, if there are multiple motors, a 120 ohm resistor needs to be connected to the end motor.***

1. First, use the latest host computer to set a baud rate of 5M for each motor.

2. Then define the id variable in the main function:
```shell
uint16_t canid1 = 0x01;
uint16_t mstid1 = 0x11;
uint16_t canid2 = 0x02;
uint16_t mstid2 = 0x12;
uint16_t canid3 = 0x03;
uint16_t mstid3 = 0x13;
uint16_t canid4 = 0x04;
uint16_t mstid4 = 0x14;
uint16_t canid5 = 0x05;
uint16_t mstid5 = 0x15;
uint16_t canid6 = 0x06;
uint16_t mstid6 = 0x16;
uint16_t canid7 = 0x07;
uint16_t mstid7 = 0x17;
uint16_t canid8 = 0x08;
uint16_t mstid8 = 0x18;
uint16_t canid9 = 0x09;
uint16_t mstid9 = 0x19;
```
3. Then define the USB-to-CANFD baudrate:

```shell
uint32_t nom_baud =1000000;//仲裁域1M波特率
uint32_t dat_baud =5000000;//数据域5M波特率
```
4. Then define a motor information container:
```shell
std::vector<damiao::DmActData> init_data;
```
5. Then fill the container with data from 9 motors:
```shell
init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
                                            .mode = damiao::MIT_MODE,
                                            .can_id=canid1,
                                            .mst_id=mstid1 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid2,
       .mst_id=mstid2 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
       .mode = damiao::MIT_MODE,
        .can_id=canid3,
        .mst_id=mstid3 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid4,
        .mst_id=mstid4 });
 
init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid5,
        .mst_id=mstid5 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid6,
        .mst_id=mstid6 });
      
init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid7,
        .mst_id=mstid7 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid8,
        .mst_id=mstid8 });

init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::MIT_MODE,
        .can_id=canid9,
        .mst_id=mstid9 });
```
6. Then initialize the motor control structure:
```shell
std::shared_ptr<damiao::Motor_Control> control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      "AA96DF2EC013B46B1BE4613798544085",&init_data);
```
***Note: "AA96DF2EC013B46B1BE4613798544085" above is my device's serial number. You need to replace it with your device's serial number. You can find your device's serial number by running the dev_sn file, as mentioned earlier.***

7. Next, the motors can be controlled through this structure by sending MIT commands to all nine motors:
```shell
control->control_mit(*control->getMotor(canid1), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid2), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid3), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid4), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid5), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid6), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid7), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid8), 0.0, 0.0, 0.0, 0.0, 0.0);
control->control_mit(*control->getMotor(canid9), 0.0, 0.0, 0.0, 0.0, 0.0);
```
8. Acquire the position, speed, torque, and time interval of the data received from the motors from the nine motors:
```shell
for(uint16_t id = 1;id<=6;id++)
{
  float pos=control->getMotor(id)->Get_Position();//电机实际位置
  float vel=control->getMotor(id)->Get_Velocity();//电机实际速度
  float tau=control->getMotor(id)->Get_tau();//电机输出力矩
  double interval=control->getMotor(id)->getTimeInterval() ; //The time interval for receiving data from the motor
  std::cerr<<"canid is: "<<id<<" pos: "<<pos<<" vel: "<<vel
          <<" effort: "<<tau<<" time(s): "<<interval<<std::endl;
}
```
9. Achieving a control loop of 1kHz:

***Note: It's better to use the function std::this_thread::sleep_until(sleep_till).***
```shell
while (1) 
{ 
  const duration desired_duration(0.001); // Calculate expected period
  auto current_time = clock::now();


  const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
  std::this_thread::sleep_until(sleep_till);    
}
```
## Receive Callback Explanation

n damiao.cpp, the function void Motor\_Control::canframeCallback(can\_value_type& value)，

This function parses the received CAN messages. It cannot be called manually; it's passed as an argument to the `usb_class` class, which then starts a thread to call this function.

You can override this function to parse your own CAN message data.
