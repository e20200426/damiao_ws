#include "protocol/damiao.h"
#include <csignal>
#include <iostream>
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);
std::atomic<float> q_target(0.0f);  // Use atomic to safely update from multiple threads

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

// Function to handle user input for target position
void getUserInput() {
    while (running) {
        std::cout << "Enter target position in radians (current: " << q_target << "): ";
        float input_position;
        char input_char;
        std::cin >> input_char;  // Capture the input first

        if (input_char == 'q' || input_char == 'Q') {
            running = false;  // Set running flag to false to exit the program
            std::cerr << "Exiting the program.\n";
            break;
        } else {
            std::cin.putback(input_char);  // Put the character back into the input stream
            std::cin >> input_position;    // Read the actual position value
            q_target = input_position;     // Update target position
        }
    }
}


int main(int argc, char** argv)
{
    using clock = std::chrono::steady_clock;
    using duration = std::chrono::duration<double>;

    std::signal(SIGINT, signalHandler);

    try 
    {
        uint16_t canid7 = 0x01;
        uint16_t mstid7 = 0x11;
        uint32_t nom_baud = 1000000;
        uint32_t dat_baud = 1000000;

        std::vector<damiao::DmActData> init_data;
        
        init_data.push_back(damiao::DmActData{
            .motorType = damiao::DM4340_48V,
            .mode = damiao::POS_VEL_MODE,
            .can_id = canid7,
            .mst_id = mstid7
        });

        std::shared_ptr<damiao::Motor_Control> control = std::make_shared<damiao::Motor_Control>(
            nom_baud, dat_baud, "E067CA134F67746CCA5451F1BE23BAD8", &init_data
        );

        auto motor = control->getMotor(canid7);

        // Switch control mode to POS_VEL
        control->switchControlMode(*motor, damiao::POS_VEL);

        // const float kp = 15.0f;      // tune
        // const float kd = 2.0f;       // tune
        // const float dq_target = 0.0f;
        // const float tau_ff = 0.0f;


        // Start the input thread to capture user input in real-time
        std::thread input_thread(getUserInput);

        while (running)
        {
            const duration desired_duration(0.01);
            auto current_time = clock::now();

            // Position control in MIT mode using the real-time adjusted q_target
            control->control_pos_vel(*motor, q_target.load(), 10.0f);

            // Apply the gearbox ratio here: 1:100 reduction
            float pos = motor->Get_Position() ; // Adjusted position
            float vel = motor->Get_Velocity() ;// gear_ratio; // Adjusted velocity
            float tau = motor->Get_tau();
            double interval = motor->getTimeInterval();
            std::cerr << "pos: " << pos << " vel: " << vel
                      << " tau: " << tau << " dt(s): " << interval << std::endl;

            const auto sleep_till = current_time + std::chrono::duration_cast<clock::duration>(desired_duration);
            std::this_thread::sleep_until(sleep_till);
        }

        // Wait for the input thread to finish before exiting
        input_thread.join();

        std::cout << std::endl << "The program exited safely." << std::endl << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: hardware interface exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
