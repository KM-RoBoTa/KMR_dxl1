/**
 ******************************************************************************
 * @file            KMR_dxlP1_reader.cpp
 * @brief           Defines the Reader class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT 
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxlP1_reader.hpp"
#include <algorithm>
#include <cstdint>

#define MAX_POS         28672
#define UINT_OVERFLOW   65535

using std::cout;
using std::endl;
using std::vector;


namespace KMR::dxlP1
{

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Previouly initialized Hal object
 */
Reader::Reader(Fields field, vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, Hal hal)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;
    m_hal = hal;
    m_ids = ids;

    m_field = field;

    getDataByteSize();
    checkMotorCompatibility(field);

    m_groupBulkReader = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);

    // Create the table to save read data
    m_dataFromMotor = new float [m_ids.size()];                          

}


/**
 * @brief Destructor
 */
Reader::~Reader()
{
    //cout << "The Dxl Reader object is being deleted" << endl;
}

/*
 *****************************************************************************
 *                             Data reading
 ****************************************************************************/

/**
 * @brief   Clear the parameters list: no motors added
 */
void Reader::clearParam()
{
    m_groupBulkReader->clearParam();
}

/**
 * @brief       Add a motor to the list of motors who will read
 * @param[in]   id ID of the motor
 * @retval      bool: true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupBulkReader->addParam(id, m_data_address, m_data_byte_size);
    return dxl_addparam_result;
}

/**
 * @brief       Read the handled fields of input motors
 * @param[in]   ids List of motors whose fields will be read 
 * @retval      void
 */
void Reader::syncRead(vector<int> ids)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = 0;
    uint8_t id;

    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<ids.size(); i++){
        id = (uint8_t)ids[i];
        dxl_addparam_result = addParam(id);
        if (dxl_addparam_result != true) {
            cout << "[KMR::dxlP1::Reader] Adding parameters failed for ID = " << ids[i] << endl;
            exit(1);
        }
    }

    // Read the motors' sensors
    dxl_comm_result = m_groupBulkReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        //exit(1);
    }

    checkReadSuccessful(ids);
    populateOutputMatrix(ids);
}


/**
 * @brief       Check if read data from motors is available
 * @param[in]   ids List of motors whose fields have just been read
 * @retval      void
 */
void Reader::checkReadSuccessful(vector<int> ids)
{
    // Check if groupsyncread data of Dyanamixel is available
    bool dxl_getdata_result = false;
    Fields field = m_field;

    for (int i=0; i<ids.size(); i++) {
        dxl_getdata_result = m_groupBulkReader->isAvailable(ids[i], m_data_address, m_data_byte_size);

        if (dxl_getdata_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", ids[i]);
            //exit(1);
        }
    }
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 * @param[in]   ids List of motors whose fields have been successfully read
 * @retval      void
 */
void Reader::populateOutputMatrix(vector<int> ids)
{
    Fields field = m_field;
    uint32_t paramData;
    float units, data;
    int id = 0, idx = 0;

    for (int i=0; i<ids.size(); i++) {
        id = ids[i];
        units = m_hal.getControlParametersFromID(id, field).unit;

        paramData = m_groupBulkReader->getData(id, m_data_address, m_data_byte_size);

        // Transform data from parametrized value to SI units
        if (field != GOAL_POS && field != PRESENT_POS &&
            field != CW_ANGLE_LIMIT && field != CCW_ANGLE_LIMIT) {
            data = paramData * units;        
        }
        else {
            // In multiturn mode, paramData overflows when position parameter < 0
            if (paramData > MAX_POS)
                paramData = paramData - UINT_OVERFLOW; 
            data = position2Angle(paramData, id, units);
        }
            
        // Save the converted value into the output matrix
        idx = getMotorIndexFromID(id);
        m_dataFromMotor[idx] = data;
    }
}


/**
 * @brief       Convert position into angle based on motor model 
 * @param[in]   position Position to be converted
 * @param[in]   id ID of the motor
 * @param[in]   units Conversion units between the position and the angle
 * @return      Angle position [rad] of the query motor
 */
float Reader::position2Angle(int32_t position, int id, float units)
{
    float angle;

    int motor_idx = m_hal.getMotorsListIndexFromID(id);
    int model = m_hal.m_motors_list[motor_idx].scanned_model;

    if (model == 1030 || model == 1000 || model == 310){
    	int Model_max_position = 4095;
        
        angle = ((float) position - Model_max_position/2) * units;
    }
    else {
        cout << "This model is unknown, cannot calculate angle from position!" << endl;
        return (1);
    }

    return angle;
}

}