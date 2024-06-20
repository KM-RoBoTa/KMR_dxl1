/**
 ******************************************************************************
 * @file            KMR_dxlP1_robot.cpp
 * @brief           Defines the BaseRobot class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include <cstdint>
#include <iostream>


#include <unistd.h>  // Provides sleep function for linux
#include "KMR_dxlP1_robot.hpp"

#define PROTOCOL_VERSION            1.0
#define ENABLE                      1
#define DISABLE                     0


using namespace std;

namespace KMR::dxlP1
{


/**
 * @brief       Constructor for BaseRobot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   port_name Name of the port handling the communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   hal Previously initialized Hal object
 */
BaseRobot::BaseRobot(vector<int> all_ids, const char *port_name, int baudrate, Hal* hal)
{
    m_hal = hal;
    m_all_IDs = all_ids;

    // Connect U2D2
    init_comm(port_name, baudrate, PROTOCOL_VERSION);

    // 2 integrated handlers: motor enabling and mode setter
    m_motor_enabler = new Writer(TRQ_ENABLE, m_all_IDs, portHandler_, packetHandler_, m_hal);
    m_CW_limit = new Writer(CW_ANGLE_LIMIT, m_all_IDs, portHandler_, packetHandler_, m_hal);
    m_CCW_limit = new Writer(CCW_ANGLE_LIMIT, m_all_IDs, portHandler_, packetHandler_, m_hal);

    // Ping each motor to validate the communication is working
    check_comm();
}

/**
 * @brief Destructor
 */
BaseRobot::~BaseRobot()
{
    // Free the dynamically allocated memory to heap
    delete m_motor_enabler;
    delete m_CW_limit;
    delete m_CCW_limit;
    delete m_torque_control;
    delete m_EEPROM_writer;
}


/**
 * @brief       Initialize the serial communication
 * @param[in]   port_name Name of the port handling communication with motors
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   protocol_version Protocol version, for the communication (U2D2)
 */
void BaseRobot::init_comm(const char *port_name, int baudrate, float protocol_version)
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(port_name);
    if (!portHandler_->openPort()) {
        cout<< "Failed to open the motors port!" <<endl;
        exit(1);
    }
    else
        cout<< "Succeed to open the motors port" <<endl;

    if (!portHandler_->setBaudRate(baudrate)) {
        cout<< "Failed to set baudrate!" <<endl;
        return ;
    }
    else
        cout<< "Succeeded to change the baudrate!" <<endl;

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

/**
 * @brief       Ping each motor to validate the communication is working
 */
void BaseRobot::check_comm()
{
    bool result = false;
    uint16_t model_number = 0;
    uint8_t dxl_error = 0;
    int motor_idx;
    int id = 0;

    cout << "Pinging motors...." << endl;

    for (int i=0; i<m_all_IDs.size(); i++) {
        id = m_all_IDs[i];
        result = packetHandler_->ping(portHandler_, id, &model_number, &dxl_error);
        if (result != COMM_SUCCESS) {
            cout << "Failed to ping, check config file and motor ID: " << id << endl;
            cout << packetHandler_->getTxRxResult(result) << endl;
            exit(1);
        }
        else {
            cout << "id: " << id << ", model number : " << model_number << endl;
            motor_idx = m_hal->getMotorsListIndexFromID(id);
            m_hal->m_motors_list[motor_idx].scanned_model = model_number; 
        }
    }
}

/**
 * @brief       Set the IDs of all motors in multiturn mode    
 * @param[in]   ids IDs of motors in multiturn
 */
void BaseRobot::setMultiturnIds(vector<int> ids)
{
    m_torque_control = new Writer(TRQ_MODE_ENABLE, ids, portHandler_, packetHandler_, m_hal);
}

/*
******************************************************************************
 *                         Enable/disable motors
 ****************************************************************************/

/**
 * @brief       Enable all the motors
 */
void BaseRobot::enableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Enable motors specified by IDs
 * @param[in]   ids List of motor ids to be enabled
 */
void BaseRobot::enableMotors(vector<int> ids)
{
    m_motor_enabler->addDataToWrite(vector<int>{ENABLE}, ids);
    m_motor_enabler->syncWrite(ids);    
}

/**
 * @brief       Disable all the motors
 */
void BaseRobot::disableMotors()
{
    m_motor_enabler->addDataToWrite(vector<int>{DISABLE}, m_all_IDs);
    m_motor_enabler->syncWrite(m_all_IDs);
}

/**
 * @brief       Disable motors specified by IDs
 * @param[in]   ids List of motor ids to be disabled
 */
void BaseRobot::disableMotors(vector<int> ids)
{
    m_motor_enabler->addDataToWrite(vector<int>{DISABLE}, ids);
    m_motor_enabler->syncWrite(ids);    
}


/*
******************************************************************************
 *                Reset necessary motors in multiturn mode
 ****************************************************************************/
/**
 * @brief       Set single motor to multiturn mode. Used for multiturn reset
 * @param[in]   id Motor id to get set to multiturn mode
 */
void BaseRobot::setMultiturnControl_singleMotor(int id)
{
    m_CW_limit->addDataToWrite(vector<float>{0}, vector<int>{id});
    m_CW_limit->syncWrite(vector<int>{id});
}

/**
 * @brief       Set single motor to position control mode. Used for multiturn reset
 * @param[in]   id Motor id to get set to control position mode
 */
void BaseRobot::setPositionControl_singleMotor(int id)
{
    m_CW_limit->addDataToWrite(vector<float>{0}, vector<int>{id});
    m_CW_limit->syncWrite(vector<int>{id});
}


/**
 * @brief       Set single motor to torque control mode. Used for multiturn reset
 * @param[in]   id Motor id to get set to multiturn mode
 * @param[in]   on_off 1/0 for enable/disable torque control mode
 */
void BaseRobot::setTorqueControl_singleMotor(int id, int on_off)
{
    m_torque_control->addDataToWrite(vector<int>{on_off}, vector<int>{id});
    m_torque_control->syncWrite(vector<int>{id});
}


/**
 * @brief       Reset multiturn motors flagged as needing a reset, with input sleeping time.
 * @param[in]   sleep_time_us Sleep time in microseconds after dis/enabling motors
 * @note        Make sure the motors had enough time to execute the goal position command before 
 *              calling this function. Failure to do so results in undefined behavior.
 */
void BaseRobot::resetMultiturnMotors(int sleep_time_us)
{
    Motor motor;
    int id;
    int reset_flag = 0;

    for(int i=0; i<m_all_IDs.size(); i++) {
        id = m_all_IDs[i];
        motor = m_hal->getMotorFromID(id);
        if (motor.toReset) {
            reset_flag = 1;
            break;
        }
    }

    if (reset_flag == 1 || reset_flag == 2) {
        disableMotors();

        for(int i=0; i<m_all_IDs.size(); i++) {
            id = m_all_IDs[i];
            motor = m_hal->getMotorFromID(id);
            if (motor.toReset == 1) {
                setPositionControl_singleMotor(id);
                m_hal->updateResetStatus(id, 2);
            }
        }

        // Need to enable the motors with the new control type for it to register
        usleep(sleep_time_us);
        enableMotors();
        usleep(sleep_time_us);
        disableMotors();

        for(int i=0; i<m_all_IDs.size(); i++) {
            id = m_all_IDs[i];
            motor = m_hal->getMotorFromID(id);
            if (motor.toReset == 2) {
                setMultiturnControl_singleMotor(id);
                m_hal->updateResetStatus(id, 0);
            }
        }
        usleep(sleep_time_us);
        enableMotors();
        usleep(sleep_time_us);
    }
}


/**
 * @brief       Reset multiturn motors flagged as needing a reset, using the default 1ms sleep time. \n 
 *              Use the overloaded function to set a custom value as argument
 * @note        Make sure the motors had enough time to execute the goal position command before 
 *              calling this function. Failure to do so results in undefined behavior.
 */
void BaseRobot::resetMultiturnMotors()
{
    resetMultiturnMotors(1000);
}


/*
******************************************************************************
 *                               EEPROM init writing
 ****************************************************************************/

/**
 * @brief       Set the input motors to multiturn mode
 * @param[in]   ids Motors to be set to multiturn mode
 */
void BaseRobot::setMultiturnMode(vector<int> ids)
{
    // Enable the motors for a bit for the previous settings the take effect
    enableMotors();
    usleep(10*1000);
    disableMotors();

    m_CW_limit->addParametersToWrite(vector<int>{4095}, ids);
    m_CW_limit->syncWrite(ids);

     // Enable the motors for a bit for the previous settings the take effect
    enableMotors();
    usleep(10*1000);
    disableMotors();   

    m_CCW_limit->addParametersToWrite(vector<int>{4095}, ids);
    m_CCW_limit->syncWrite(ids);

    // Enable the motors for a bit for the previous settings the take effect
    enableMotors();
    usleep(10*1000);
    disableMotors();   
}


/**
 * @brief       Set the minimum voltage of motors
 * @param[in]   minVoltages Min. allowed voltages in motors
 */                                 
void BaseRobot::setMinVoltage(vector<float> minVoltages)
{
    m_EEPROM_writer = new Writer(MIN_VOLT_LIMIT, 
                                m_all_IDs, portHandler_, packetHandler_, m_hal);

    m_EEPROM_writer->addDataToWrite(minVoltages, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);
    usleep(30*1000);

    // Free the memory
    delete m_EEPROM_writer;
    m_EEPROM_writer = nullptr;
}

/**
 * @brief       Set the maximum voltage of motors
 * @param[in]   minVoltages Max. allowed voltages in motors
 */                                 
void BaseRobot::setMaxVoltage(vector<float> maxVoltages)
{
    m_EEPROM_writer = new Writer(MAX_VOLT_LIMIT, 
                                m_all_IDs, portHandler_, packetHandler_, m_hal);

    m_EEPROM_writer->addDataToWrite(maxVoltages, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);
    usleep(30*1000);

    // Free the memory
    delete m_EEPROM_writer;
    m_EEPROM_writer = nullptr;
}

/**
 * @brief       Set the minimum position of motors
 * @param[in]   minPositions Min. positions for motors (lower saturation) 
 */                                 
void BaseRobot::setMinPosition(vector<float> minPositions)
{
    m_EEPROM_writer = new Writer(CW_ANGLE_LIMIT, 
                                m_all_IDs, portHandler_, packetHandler_, m_hal);

    m_EEPROM_writer->addDataToWrite(minPositions, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);
    usleep(30*1000);

    // Free the memory
    delete m_EEPROM_writer;
    m_EEPROM_writer = nullptr;
}

/**
 * @brief       Set the maximum position of motors
 * @param[in]   maxPositions Max. positions for motors (upper saturation) 
 */                                 
void BaseRobot::setMaxPosition(vector<float> maxPositions)
{
    m_EEPROM_writer = new Writer(CCW_ANGLE_LIMIT, 
                                m_all_IDs, portHandler_, packetHandler_, m_hal);

    m_EEPROM_writer->addDataToWrite(maxPositions, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);
    usleep(30*1000);

    // Free the memory
    delete m_EEPROM_writer;
    m_EEPROM_writer = nullptr;   
}

/**
 * @brief   Set the return delay to all motors
 */
void BaseRobot::setAllDelay(int val)
{
    m_EEPROM_writer = new Writer(RETURN_DELAY, 
                                m_all_IDs, portHandler_, packetHandler_, m_hal);

    m_EEPROM_writer->addDataToWrite(vector<int>{val}, m_all_IDs);
    m_EEPROM_writer->syncWrite(m_all_IDs);
    usleep(30*1000);

    // Free the memory
    delete m_EEPROM_writer;
    m_EEPROM_writer = nullptr;   
}


}
