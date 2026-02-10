#include "protocol/damiao.h"
#include <csignal>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

int main(int argc, char** argv)
{
  using clock = std::chrono::steady_clock;
  using duration = std::chrono::duration<double>;

  std::signal(SIGINT, signalHandler);

  try 
  {
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
      
      uint32_t nom_baud =1000000;
      uint32_t dat_baud =1000000;

      std::vector<damiao::DmActData> init_data;
    
      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340_48V,
        .mode = damiao::POS_VEL_MODE,
        .can_id=canid1,
        .mst_id=mstid1 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340_48V,
      .mode = damiao::POS_VEL_MODE,
      .can_id=canid2,
      .mst_id=mstid2 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340_48V,
      .mode = damiao::POS_VEL_MODE,
      .can_id=canid3,
      .mst_id=mstid3 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4340_48V,
      .mode = damiao::POS_VEL_MODE,
      .can_id=canid4,
      .mst_id=mstid4 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
      .mode = damiao::POS_VEL_MODE,
      .can_id=canid5,
      .mst_id=mstid5 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
      .mode = damiao::POS_VEL_MODE,
      .can_id=canid6,
      .mst_id=mstid6 });

      init_data.push_back(damiao::DmActData{.motorType = damiao::DM4310,
        .mode = damiao::POS_VEL_MODE,
        .can_id=canid7,
        .mst_id=mstid7 });

     std::shared_ptr<damiao::Motor_Control> control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      "E067CA134F67746CCA5451F1BE23BAD8",&init_data);

     // List of all motor CAN IDs
     std::vector<uint16_t> motor_ids = {
        canid1, canid2, canid3, canid4, canid5, canid6, canid7
     };

     // Switch control mode + set zero for all motors
     for (auto id : motor_ids)
     {
         auto motor = control->getMotor(id);

         // Ensure correct mode
         control->switchControlMode(*motor, damiao::POS_VEL);
         std::this_thread::sleep_for(std::chrono::milliseconds(20));

         // Set current position as zero
         control->set_zero_position(*motor);
         std::this_thread::sleep_for(std::chrono::milliseconds(50));
         std::cout << "Motor CAN ID " << id << " zeroed." << std::endl;
     }

     auto motor = control->getMotor(canid1);

     while (running)
     {
        const duration desired_duration(0.01);
        auto current_time = clock::now();

        // Position control in POS_VEL mode:
        // control->control_pos_vel(*motor, q_target.load(), 10.0f);

        float pos = motor->Get_Position();
        float vel = motor->Get_Velocity();
        float tau = motor->Get_tau();
        double interval = motor->getTimeInterval();
        std::cerr << "pos: " << pos << " vel: " << vel
                  << " tau: " << tau << " dt(s): " << interval << std::endl;

        const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
        std::this_thread::sleep_until(sleep_till);
     }

      std::cout <<  std::endl<<"The program exited safely." << std::endl<< std::endl;
  }
  catch (const std::exception& e) {
      std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
      return 1;
  }

  return 0;
}
