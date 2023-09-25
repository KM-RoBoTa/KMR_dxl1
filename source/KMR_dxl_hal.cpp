/**
 ******************************************************************************
 * @file            KMR_dxl_hal.cpp
 * @brief           Defines the Hal class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT  
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxl_hal.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <cstdint>

using namespace std;


namespace YAML
{

/**
 * @brief       Overload YAML::Node.as to be usable with our Data_node structure: \n
 *              Convert YAML::Node to Data_node
 * @param[in]   node YAML:Node read by the YAML parser
 * @param[out]  data_node Instance of Data_node to store the info gotten from node
 * @retval      void
 */
template <>
struct convert<KMR::dxl::Data_node>
{

    static bool decode(const Node &node, KMR::dxl::Data_node &data_node)
    {

        data_node.field_name = node["field"].as<std::string>();
        data_node.address = node["address"].as<int>();
        data_node.length = node["length"].as<int>();
        data_node.unit = node["unit"].as<float>();

        return true;
    }
};

/**
 * @brief       Overload YAML::Node.as to be usable with our Motor_node structure: \n
 *              Convert YAML::Node to Motor_node
 * @param[in]   node YAML:Node read by the YAML parser
 * @param[out]  motor_node Instance of Motor_node to store the info gotten from node
 * @retval      void
 */
template <>
struct convert<KMR::dxl::Motor_node>
{

    static bool decode(const Node &node, KMR::dxl::Motor_node &motor_node)
    {
        motor_node.id = node["ID"].as<int>();
        motor_node.model_name = node["model"].as<string>();
        motor_node.multiturn = node["multiturn"].as<int>();
        return true;
    }
};


/**
 * @brief       Overload YAML::Node.as to be usable with our Control_modes structure: \n
 *              Convert YAML::Node to Control_modes
 * @param[in]   node YAML:Node read by the YAML parser
 * @param[out]  motor_node Instance of Control_modes to contain the info gotten from node
 * @retval      void
 */
template <>
struct convert<KMR::dxl::Control_modes>
{

    static bool decode(const Node &node, KMR::dxl::Control_modes &control_modes)
    {

        control_modes.current_control = (uint8_t) node["current_control"].as<int>();
        control_modes.current_based_position_control = (uint8_t) node["current_based_position_control"].as<int>();
        control_modes.multiturn_control = (uint8_t) node["multiturn_control"].as<int>();
        control_modes.velocity_control = (uint8_t) node["velocity_control"].as<int>();
        control_modes.position_control = (uint8_t) node["position_control"].as<int>();
        control_modes.PWM_control = (uint8_t) node["PWM_control"].as<int>();

        return true;
    }
};

}

namespace KMR::dxl
{
    
/**
 * @brief       Constructor for Hal
 */
Hal::Hal()
{
    m_tot_nbr_motors = -1;
    m_control_table = new Motor_data_field *[NBR_MODELS];
    for (int i = 0; i < NBR_MODELS; i++)
        m_control_table[i] = new Motor_data_field[NBR_FIELDS];

    m_controlModesPerModel = new Control_modes[NBR_MODELS];

    cout << "[KMR::dxl] Hal created" << endl;
}

/**
 * @brief       Initialize the hal: parse motor config file and create the control table. 
 *              To call immediately after the constructor   
 * @param[in]   motor_config_file Configuration file of the motors in the project
 * @return      Vector of all motor IDs 
 */
vector<int> Hal::init(char *motor_config_file, char* path_to_KMR_dxl)
{
    // Parse the motor config specific to the current project
    parse_motor_config(motor_config_file);

    // Create the control table for all models
    populate_control_table(path_to_KMR_dxl);

    // Extract the list of motor IDs
    get_ID_list_from_motors_list();

    // Save operating modes values to motors
    saveControlValuesToMotors();

    return m_all_IDs;
}


/**
 * @brief       Destructor for Hal
 */
Hal::~Hal()
{
    cout << "[KMR::dxl] The Hal object is being deleted" << endl;
}


/*****************************************************************************
 *                   Creation of the control table
 ****************************************************************************/

/**
 * @brief       Populate the control table's data fields for all motor models in the project
 * @param[in]   path_to_KMR_dxl Path from the working directory (build) to this library's folder
 * @retval      void
 */
void Hal::populate_control_table(char* path_to_KMR_dxl)
{
    Data_node data_node;
    Motor_data_field motor_data_field;
    Fields col;

    // For each motor model, open its config file and populate the control table
    for (int i = 0; i < m_unique_motor_models_list.size(); i++)
    {
        string config_file = (string)path_to_KMR_dxl + (string)"/config/motor_models/"
                             + m_unique_motor_models_list[i];

        // Open the yaml config file
        YAML::Node config = YAML::LoadFile(config_file);
        cout << "[KMR::dxl] Motor model file open: " << config_file << endl;

        // Read and convert the first line to get the motor model name
        string motor_model_string = config["model_name"].as<string>();
        Motor_models motor_model = string2Motors_models(motor_model_string);

        // Read the values to set control modes
        Control_modes control_modes = config["operating_modes"][0].as<Control_modes>();
        m_controlModesPerModel[motor_model] = control_modes;

        // Read the motor_data nodes: get the name, address, length and unit of each data field
        for (int j = 0; j < config["motor_data"].size(); j++)
        {
            data_node = config["motor_data"][j].as<Data_node>();

            // Convert the read node into our structures
            col = string2Fields(data_node.field_name);
            dataNode2Motor_data_field(data_node, motor_data_field);

            // Populate the control table
            m_control_table[motor_model][col] = motor_data_field;
        }
    }
}

/*****************************************************************************
 *          Conversions from nodes/strings to our structures/enumerates
 ****************************************************************************/

/**
 * @brief       Convert a string to Motors_models enumerate
 * @attention   This function needs to be updated if new motor model added, as well as the enum
 * @param[in]   str String to be converted into the enumerate value
 * @retval      Motors_models enumerate value
 */
Motor_models Hal::string2Motors_models(const string &str)
{
    if (str == "MX_64R")
        return MX_64R;
    else
        return UNDEF_M;
    /* else if(str == "TUESDAY") return TUESDAY;
    else if(str == "WEDNESDAY") return WEDNESDAY;
    else if(str == "THURSDAY") return THURSDAY;
    else if(str == "FRIDAY") return FRIDAY;
    else if(str == "SATURDAY") return SATURDAY;
    else return SUNDAY; */
}

/**
 * @brief       Convert a string to Fields enumerate
 * @param[in]   str String to be converted into the enumerate value
 * @retval      Fields enumerate value
 */
Fields Hal::string2Fields(const string &str)
{
    if (str == "MODEL_NBR")
        return MODEL_NBR;
    else if (str == "MODEL_INFO")
        return MODEL_INFO;
    else if (str == "FIRMWARE")
        return FIRMWARE;
    else if (str == "ID")
        return ID;
    else if (str == "BAUDRATE")
        return BAUDRATE;
    else if (str == "RETURN_DELAY")
        return RETURN_DELAY;
    else if (str == "DRIVE_MODE")
        return DRIVE_MODE;
    else if (str == "RETURN_DELAY")
        return RETURN_DELAY;
    else if (str == "OP_MODE")
        return OP_MODE;
    else if (str == "SHADOW_ID")
        return SHADOW_ID;
    else if (str == "PROTOCOL")
        return PROTOCOL;
    else if (str == "HOMING_OFFSET")
        return HOMING_OFFSET;
    else if (str == "MOVE_THRESHOLD")
        return MOVE_THRESHOLD;
    else if (str == "TEMP_LIMIT")
        return TEMP_LIMIT;
    else if (str == "MAX_VOLT_LIMIT")
        return MAX_VOLT_LIMIT;
    else if (str == "MIN_VOLT_LIMIT")
        return MIN_VOLT_LIMIT;
    else if (str == "PWM_LIMIT")
        return PWM_LIMIT;
    else if (str == "CURRENT_LIMIT")
        return CURRENT_LIMIT;
    else if (str == "ACC_LIMIT")
        return ACC_LIMIT;
    else if (str == "VEL_LIMIT")
        return VEL_LIMIT;
    else if (str == "MAX_POS_LIMIT")
        return MAX_POS_LIMIT;
    else if (str == "MIN_POS_LIMIT")
        return MIN_POS_LIMIT;
    else if (str == "SHUTDOWN")
        return SHUTDOWN;
    else if (str == "TRQ_ENABLE")
        return TRQ_ENABLE;
    else if (str == "LED")
        return LED;
    else if (str == "STATUS_RETURN")
        return STATUS_RETURN;
    else if (str == "REGISTERED")
        return REGISTERED;
    else if (str == "HARDWARE_ERROR")
        return HARDWARE_ERROR;
    else if (str == "VEL_I_GAIN")
        return VEL_I_GAIN;
    else if (str == "VEL_P_GAIN")
        return VEL_P_GAIN;
    else if (str == "POS_D_GAIN")
        return POS_D_GAIN;
    else if (str == "POS_I_GAIN")
        return POS_I_GAIN;
    else if (str == "POS_P_GAIN")
        return POS_P_GAIN;
    else if (str == "FF_2ND_GAIN")
        return FF_2ND_GAIN;
    else if (str == "FF_1ST_GAIN")
        return FF_1ST_GAIN;
    else if (str == "BUS_WATCHDOG")
        return BUS_WATCHDOG;
    else if (str == "GOAL_PWM")
        return GOAL_PWM;
    else if (str == "GOAL_CURRENT")
        return GOAL_CURRENT;
    else if (str == "GOAL_VELOCITY")
        return GOAL_VELOCITY;
    else if (str == "PROFILE_ACC")
        return PROFILE_ACC;
    else if (str == "PROFILE_VEL")
        return PROFILE_VEL;
    else if (str == "GOAL_POS")
        return GOAL_POS;
    else if (str == "REALTIME_TICK")
        return REALTIME_TICK;
    else if (str == "MOVING")
        return MOVING;
    else if (str == "MOVING_STATUS")
        return MOVING_STATUS;
    else if (str == "PRESENT_PWM")
        return PRESENT_PWM;
    else if (str == "PRESENT_CURRENT")
        return PRESENT_CURRENT;
    else if (str == "PRESENT_VEL")
        return PRESENT_VEL;
    else if (str == "PRESENT_POS")
        return PRESENT_POS;
    else if (str == "VEL_TRAJECTORY")
        return VEL_TRAJECTORY;
    else if (str == "POS_TRAJECTORY")
        return POS_TRAJECTORY;
    else if (str == "PRESENT_INPUT_VOLT")
        return PRESENT_INPUT_VOLT;
    else if (str == "PRESENT_TEMP")
        return PRESENT_TEMP;
    else if (str == "INDIR_ADD_1")
        return INDIR_ADD_1;
    else if (str == "INDIR_DATA_1")
        return INDIR_DATA_1;
    else if (str == "INDIR_ADD_2")
        return INDIR_ADD_2;
    else if (str == "INDIR_DATA_2")
        return INDIR_DATA_2;
    else
        return UNDEF_F;
}

/**
 * @brief       Convert a Data_node instance to a Motor_data_field instance
 * @param[in]   data_node Data_node instance to be converted
 * @param[out]  motor_data_field Motor_data_field instance to store the info from the node
 * @retval      void
 */
void Hal::dataNode2Motor_data_field(Data_node &data_node, Motor_data_field &motor_data_field)
{
    motor_data_field.address = (std::uint8_t)data_node.address;
    motor_data_field.length = (std::uint8_t)data_node.length;
    motor_data_field.unit = data_node.unit;
}

/**
 * @brief       Convert a Motor_node instance to a Motor instance
 * @param[in]   motor_node Motor_node instance to be converted
 * @param[out]  motor Motor instance to store the info from the node
 * @retval      void
 */
void Hal::motorNode2Motor(Motor_node &motor_node, Motor &motor)
{
    motor.id = motor_node.id;
    motor.model = string2Motors_models(motor_node.model_name);
    motor.multiturn = motor_node.multiturn;
}


/*****************************************************************************
 *                Parsing of project-specific config files
 ****************************************************************************/

/**
 * @brief       Parse the motor configuration file and populate the list of motors
 * @param[in]   config_file Yaml config file for the motors in the robot
 * @return      void
 */
void Hal::parse_motor_config(char *config_file)
{
    Motor_node motor_node;
    Motor motor;

    // Open the yaml config file
    YAML::Node config = YAML::LoadFile(config_file);
    cout << "[KMR::dxl] Project motor config file open: " << config_file << endl;

    // Read and convert the first line to get the number of motors in the robot
    m_tot_nbr_motors = config["nbr_motors"].as<int>();
    if (m_tot_nbr_motors != config["motors"].size()) {
        cout << "[KMR::dxl] ERROR in the number of motors in the config file!!" << endl;
        exit(1);
    }

    // Create the list of motors (NB: had to use malloc because the struct. does not have a constructor)
    m_motors_list = (Motor *)malloc(m_tot_nbr_motors * sizeof(Motor));

    // Read the motors nodes: get the ID, model and multiturn mode for each motor
    for (int i = 0; i < config["motors"].size(); i++)
    {
        motor_node = config["motors"][i].as<Motor_node>();

        // Convert the read node and update the list of used models
        motorNode2Motor(motor_node, motor);
        update_unique_models_list(motor_node.model_name);

        // Populate the list of motors
        m_motors_list[i] = motor;
    }
}

/**
 * @brief       Create the list of unique motor models used in the robot
 * @param[in]   motor_model_string Model of the currently querried motor
 * @retval      void
 */
void Hal::update_unique_models_list(string motor_model_string)
{
    string filename = motor_model_string + (string) ".yaml";
    bool model_in_list = false;

    for (int i = 0; i < m_unique_motor_models_list.size(); i++)
    {
        if (m_unique_motor_models_list[i] == filename)
        {
            model_in_list = true;
            break;
        }
    }

    if (model_in_list == false)
        m_unique_motor_models_list.push_back(filename);
}



/**
 * @brief       Extract the list of all motor IDs from the motors list
 * @return      void
 */
void Hal::get_ID_list_from_motors_list()
{
    m_all_IDs = vector<int> (m_tot_nbr_motors);

    for (int i = 0; i < m_tot_nbr_motors; i++)
        m_all_IDs[i] = m_motors_list[i].id;
}


/*****************************************************************************
 *                     Query functions from outside
 ****************************************************************************/

/**
 * @brief       Get control parameters of a specific control field from motor ID
 * @note        Vital function
 * @param[in]   id ID of the query motor
 * @param[in]   field Control field of the query
 * @retval      Control parameters of the query field
 */
Motor_data_field Hal::getControlParametersFromID(int id, Fields field)
{
    Motor_models model = getModelFromID(id);
    Motor_data_field params = m_control_table[model][field];

    return params;
}

/**
 * @brief       Get motor model from motor ID
 * @param[in]   id ID of the query motor
 * @retval      Model of the query motor
 */
Motor_models Hal::getModelFromID(int id)
{
    Motor_models motor_model = NBR_MODELS;

    for (int i = 0; i < m_tot_nbr_motors; i++)
    {
        if (m_motors_list[i].id == id)
        {
            motor_model = m_motors_list[i].model;
            break;
        }
    }
    return motor_model;
}

/**
 * @brief       Get a motor's list index from its ID
 * @param[in]   id ID of the query motor
 * @retval      Index of the query motor
 */
int Hal::getMotorsListIndexFromID(int id)
{
    int i;
    for (i=0; i < m_tot_nbr_motors; i++)
    {
        if (m_motors_list[i].id == id)
        {
            break;
        }
    }

    return i;
}

/**
 * @brief       Get a motor's info structure from motor ID
 * @param[in]   id ID of the query motor
 * @retval      The Motor structure of the query motor
 */
Motor Hal::getMotorFromID(int id)
{
    int motor_idx = getMotorsListIndexFromID(id);
    Motor motor = m_motors_list[motor_idx];

    return motor;
}


/*****************************************************************************
 *                             Misc. functions
 ****************************************************************************/

/**
 * @brief       Update the offsets to access empty indirect addresses for a given motor
 * @param[in]   id ID of the query motor
 * @param[in]   data_length Byte length of the data that was just assigned an indirect address
 * @param[in]   field_name Type of field that was just assigned an indirect address (address or data)
 * @retval      void
 */
void Hal::addMotorOffsetFromID(int id, uint8_t data_length, std::string field_name)
{
    int motor_idx = getMotorsListIndexFromID(id);

    if (field_name == "indir_address_offset"){
        m_motors_list[motor_idx].indir_address_offset += data_length;
    }
    else if (field_name == "indir_data_offset")
        m_motors_list[motor_idx].indir_data_offset += data_length;
    else{
        cout << "Cannot change that motor field!" << endl;
        exit(1);
    }

}

/**
 * @brief       Update a motor's "to reset" status in multiturn mode
 * @param[in]   id ID of the query motor
 * @param[in]   status Boolean: 1 if need to reset, 0 if not
 * @retval      void
 */
void Hal::updateResetStatus(int id, int status)
{
    int idx = getMotorsListIndexFromID(id);
    m_motors_list[idx].toReset = status;
}


/**
 * @brief       For each motor, save all operating modes control values. \n 
 *              Needed for resetting in multiturn mode
 * @retval      void
 */
void Hal::saveControlValuesToMotors()
{
    int idx, id, model;

    for (int i=0; i<m_tot_nbr_motors; i++){
        id = m_all_IDs[i];
        idx = getMotorsListIndexFromID(id);
        model = m_motors_list[idx].model;

        m_motors_list[idx].control_modes.current_based_position_control = 
                            m_controlModesPerModel[model].current_based_position_control;
        m_motors_list[idx].control_modes.position_control = m_controlModesPerModel[model].position_control;
        m_motors_list[idx].control_modes.current_control = m_controlModesPerModel[model].current_control;
        m_motors_list[idx].control_modes.multiturn_control = m_controlModesPerModel[model].multiturn_control;
        m_motors_list[idx].control_modes.PWM_control = m_controlModesPerModel[model].PWM_control;
        m_motors_list[idx].control_modes.velocity_control= m_controlModesPerModel[model].velocity_control;
    }
}
       

}