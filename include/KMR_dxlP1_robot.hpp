/**
 ******************************************************************************
 * @file            KMR_dxlP1_robot.hpp
 * @brief           Header for the KMR_dxlP1_robot.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXLP1_ROBOT_HPP
#define KMR_DXLP1_ROBOT_HPP

#include "KMR_dxlP1_writer.hpp"
#include "KMR_dxlP1_reader.hpp"

namespace KMR::dxlP1
{

/**
 * @brief   Class that defines a base robot, to be inherited by a robot class in the project
 * @details This class contains base necessities for handling a robot with dynamixel motors. \n 
 *          It provides functions to enable/disable motors, as well as to reset motors in multiturn. \n 
 *          The user needs to create handlers they need (Writers and Readers) for their specific
 *          application, as well as their respective reading/writing functions
 */
class BaseRobot {
    protected:
        dynamixel::PortHandler   *portHandler_;
        dynamixel::PacketHandler *packetHandler_;

        Writer *m_motor_enabler;
        Writer *m_CW_limit;
        Writer *m_CCW_limit;
        Writer *m_torque_control;

        void init_comm(const char *port_name, int baudrate, float protocol_version);
        void check_comm();
        void setMultiturnControl_singleMotor(int id);
        void setPositionControl_singleMotor(int id);
        void setTorqueControl_singleMotor(int id, int on_off);

        
    public:
        int *scanned_motor_models;  // Dynamixel-defined model numbers of motors in the robot
        Hal m_hal;  // to put private? @todo
        std::vector<int> m_all_IDs; // All motor IDs in the robot

        BaseRobot(std::vector<int> all_ids, const char *port_name, int baudrate, Hal hal);
        ~BaseRobot();
       
        void enableMotors();
        void enableMotors(std::vector<int> ids);
        void disableMotors();
        void disableMotors(std::vector<int> ids);
        void resetMultiturnMotors();
        void resetMultiturnMotors(int sleep_time_us);
};

}
#endif
