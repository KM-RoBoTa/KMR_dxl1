/**
 *****************************************************************************
 * @file            KMR_dxl_hal.hpp
 * @brief           Header for KMR_dxl_hal.cpp file
 *****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 *****************************************************************************
 */

#ifndef KMR_DXL_HAL_HPP
#define KMR_DXL_HAL_HPP

#include <string>
#include <iostream>
#include <cstdint>
#include <vector>

namespace KMR::dxl
{

/**
 * @brief   Structure saving the info of a data field
 */
struct Motor_data_field {
    std::uint8_t address;
    std::uint8_t length;
    float unit;
};


/**
 * @brief   Structure used to store info from a YAML::Node during parsing the motors' control tables
 */
struct Data_node {
    std::string field_name;
    int address;
    int length;
    float unit;
};

/**
 * @brief   Structure used to store info from a YAML::Node during motor config parsing
 */
struct Motor_node {
    std::string model_name;
    int id;
    int multiturn;
};

/**
 * @brief       Enumerate of all dynamixel motor models (not necessarily used in the project)
 * @attention   If a new model is added, this enum needs to be updated,
 *              as well as the function "string2Motors_models" in the .cpp
 */
enum Motor_models
{
    MX_64R, NBR_MODELS, UNDEF_M
};

/**
 * @brief       Exhaustive list of all possible control modes for Dynamixel motors
 */
struct Control_modes {
    uint8_t current_control;
    uint8_t velocity_control;
    uint8_t position_control;
    uint8_t multiturn_control;    
    uint8_t current_based_position_control;
    uint8_t PWM_control;
};

/**
 * @brief   Structure saving the info of a single motor: both config-wise (ID, model...)
 *          and specific to the project (occupied indirect addresses, reset status...)
 */
struct Motor {
    int id;
    Motor_models model;
    int multiturn;
    Control_modes control_modes;

    uint8_t indir_address_offset = 0;
    uint8_t indir_data_offset = 0;
    int scanned_model = 0;
    int toReset = 0;
};


/**
 * @brief   Enumerate of all data fields in a dynamixel motor
 */
enum Fields
{
    MODEL_NBR, MODEL_INFO, FIRMWARE, ID, BAUDRATE, RETURN_DELAY, DRIVE_MODE, OP_MODE,
    SHADOW_ID, PROTOCOL, HOMING_OFFSET, MOVE_THRESHOLD, TEMP_LIMIT, MAX_VOLT_LIMIT,
    MIN_VOLT_LIMIT, PWM_LIMIT, CURRENT_LIMIT, ACC_LIMIT, VEL_LIMIT, MAX_POS_LIMIT, 
    MIN_POS_LIMIT, SHUTDOWN,
    TRQ_ENABLE, LED, STATUS_RETURN, REGISTERED, HARDWARE_ERROR, VEL_I_GAIN, VEL_P_GAIN, POS_D_GAIN,
    POS_I_GAIN, POS_P_GAIN, FF_2ND_GAIN, FF_1ST_GAIN, BUS_WATCHDOG, GOAL_PWM, GOAL_CURRENT, GOAL_VELOCITY,
    PROFILE_ACC, PROFILE_VEL, GOAL_POS, REALTIME_TICK, MOVING, MOVING_STATUS, PRESENT_PWM,
    PRESENT_CURRENT, PRESENT_VEL, PRESENT_POS, VEL_TRAJECTORY, POS_TRAJECTORY, PRESENT_INPUT_VOLT, PRESENT_TEMP,
    INDIR_ADD_1, INDIR_DATA_1, INDIR_ADD_2, INDIR_DATA_2,
    NBR_FIELDS, UNDEF_F
};



/**
 * @brief       Hardware abstraction layer for Dynamixel motors
 * @details     The lowest-level element in the library. The Hal class serves primarily as
 *              an abstraction layer, providing high-level functions to get the Dynamixel control
 *              table addresses by creating a control table. \n 
 *              It also parses the project's motors configuration file.
 */
class Hal {
private:
    std::vector<std::string> m_unique_motor_models_list;   // List of unique motor models used in the robot

    void populate_control_table(char* path_to_KMR_dxl);
    void parse_motor_config(char* config_file);
    Motor_models string2Motors_models(const std::string& str);
    Fields string2Fields(const std::string& str);
    void dataNode2Motor_data_field(Data_node& data_node, Motor_data_field& motor_data_field);
    void motorNode2Motor(Motor_node& motor_node, Motor& motor);
    void update_unique_models_list(std::string motor_model_string);
    Motor_models getModelFromID(int id);
    void saveControlValuesToMotors();

public:
    int m_tot_nbr_motors;   // Number of motors used in the robot
    Motor* m_motors_list;     // List containing all motors' info 
    std::vector<int> m_all_IDs;  // All motor IDs in the robot
    Motor_data_field** m_control_table;   // Table containing all Dynamixel control values for every model
    Control_modes* m_controlModesPerModel; // List of control modes values for each Dxl model

    Hal();
    ~Hal();
    std::vector<int> init(char* motor_config_file, char* path_to_KMR_dxl);
    void get_ID_list_from_motors_list();
    Motor_data_field getControlParametersFromID(int id, Fields field); 
    int getMotorsListIndexFromID(int id);
    Motor getMotorFromID(int id);
    void addMotorOffsetFromID(int id, uint8_t data, std::string field_name);
    void updateResetStatus(int id, int status);
};

}


#endif

