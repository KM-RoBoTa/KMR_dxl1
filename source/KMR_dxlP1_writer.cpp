/**
 ******************************************************************************
 * @file            KMR_dxlP1_writer.cpp
 * @brief           Defines the Writer class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxlP1_writer.hpp"
#include <algorithm>
#include <cstdint>

#define MULTITURN_MAX   6144
#define MULTITURN_MIN   -2048

using std::cout;
using std::endl;
using std::vector;

using namespace std;

namespace KMR::dxlP1
{

/**
 * @brief       Constructor for a Writer handler
 * @param[in]   list_fields List of fields to be handled by the writer
 * @param[in]   ids Motors to be handled by the writer
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Previouly initialized Hal object
 */
Writer::Writer(Fields field, vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, Hal hal)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;
    m_hal = hal;
    m_ids = ids;
    m_field = field;

    getDataByteSize();
    checkMotorCompatibility(field);

    m_groupSyncWriter = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, m_data_address, m_data_byte_size);

    // Create the table to save parametrized data (to be read or sent)
    m_dataParam = new uint8_t *[m_ids.size()];
    for (int i=0; i<m_ids.size(); i++)
        m_dataParam[i] = new uint8_t[m_data_byte_size];

}

/**
 * @brief Destructor
 */
Writer::~Writer()
{
    //cout << "The Dxl Writer object is being deleted" << endl;
}


/*
 *****************************************************************************
 *                             Data writing
 ****************************************************************************/

/**
 * @brief   Clear the parameters list
 */
void Writer::clearParam()
{
    m_groupSyncWriter->clearParam();
}

/**
 * @brief       Add data to be written to a motor
 * @param[in]   id ID of the motor
 * @param[in]   data Parametrized data to be sent to the motor
 * @retval      bool: true if data-to-send added to the list successfully
 */
bool Writer::addParam(uint8_t id, uint8_t* data)
{
    bool dxl_addparam_result = m_groupSyncWriter->addParam(id, data);
    return dxl_addparam_result;
}

/**
 * @brief       Send the previously prepared data with addDataToWrite to motors
 * @param[in]   ids List of motors who will receive data
 * @retval      void
 */
void Writer::syncWrite(vector<int> ids)
{
    bool dxl_addparam_result;
    int dxl_comm_result = COMM_TX_FAIL;   
    int id, motor_idx;

    clearParam();

    for(int i=0; i<ids.size(); i++) {
        id = ids[i];
        motor_idx = getMotorIndexFromID(id);

        dxl_addparam_result = addParam((uint8_t) id, m_dataParam[motor_idx]);

        if (dxl_addparam_result != true) {
            cout << "[KMR::dxlP1::Writer] Adding parameters failed for ID = " << id << endl;
            exit(1);
        }
    }

    // Send the packet
    dxl_comm_result = m_groupSyncWriter->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;

}

                                                                                                                                                                                     
/**
 * @brief       Convert angle input into position data based on motor model provided
 * @param[in]   angle Angle to be converted, in rad
 * @param[in]   id ID of the motor to receive the position input
 * @return      Position value corresponding to the joint angle (0 degrees or rad -> mid-position)
 */
int Writer::angle2Position(float angle, int id)
{
	int position = 2048;
    int motor_idx = m_hal.getMotorsListIndexFromID(id);
    int model = m_hal.m_motors_list[motor_idx].scanned_model;
    float units = m_hal.getControlParametersFromID(id, GOAL_POS).unit;
    Motor motor = m_hal.getMotorFromID(id);
    int toReset = m_hal.m_motors_list[motor_idx].toReset;

    if (model == 1030 || model == 1000 || model == 310){
    	int Model_max_position = 4095;
        int Model_min_position = 0;
		position = angle/units + Model_max_position/2 + 0.5;

        if (!motor.multiturn)
            bindParameter(Model_min_position, Model_max_position, position);
        else {
            if (multiturnOverLimit(position))
                m_hal.updateResetStatus(id, 1);


            // Force values (used for motor multiturn resetting)
            else if (toReset == 1)  // Need to set to join control mode
                position = 0;
            else if (toReset == 2) // Need to set to multiturn control mode
                position = 4095;
        }
    }
    else {
        cout << "This model is unknown, cannot calculate position from angle!" << endl;
        return (1);
    }

    return position;
}

/**
 * @brief           Saturate input value between input limits
 * @param[in]       lower_bound Min. value the input can take
 * @param[in]       upper_bound Max. value the input can take
 * @param[in/out]   param Input value to be saturated
 * @return          void
 */
void Writer::bindParameter(int lower_bound, int upper_bound, int& param)
{
    if (param > upper_bound) 
        param = upper_bound;

    else if (param < lower_bound)
        param = lower_bound; 

}


/**
 * @brief       Save a parametrized data into the general table
 * @param[in]   data Parametrized data to be sent to motor
 * @param[in]   motor_idx Index of the motor
 * @param[in]   field_idx Index of the field (type of data)
 * @param[in]   field_length Byte size of the data
 * @retval      void
 */
void Writer::populateDataParam(int32_t data, int motor_idx, int field_length)
{
    if (field_length == 4) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][1] = DXL_HIBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][2] = DXL_LOBYTE(DXL_HIWORD(data));
        m_dataParam[motor_idx][3] = DXL_HIBYTE(DXL_HIWORD(data));
    }
    else if (field_length == 2) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
        m_dataParam[motor_idx][1] = DXL_HIBYTE(DXL_LOWORD(data));
    }
    else if (field_length == 1) {
        m_dataParam[motor_idx][0] = DXL_LOBYTE(DXL_LOWORD(data));
    }
    else
        cout<< "Wrong number of parameters to populate the parametrized matrix!" <<endl;
}


/**
 * @brief       Check if the goal position will place the motor over a full turn
 * @param[in]   position Parametrized goal position of a motor
 * @retval      Boolean: 1 if over a full turn, 0 otherwise
 */
bool Writer::multiturnOverLimit(int position)
{
    if (position > MULTITURN_MAX || position < MULTITURN_MIN)
        return true;
    else
        return false;
}



}