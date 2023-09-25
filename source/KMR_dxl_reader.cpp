/**
 ******************************************************************************
 * @file            KMR_dxl_reader.cpp
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

#include "KMR_dxl_reader.hpp"
#include <algorithm>
#include <cstdint>

using std::cout;
using std::endl;
using std::vector;


namespace KMR::dxl
{

/**
 * @brief       Constructor for a Reader handler
 * @param[in]   list_fields List of fields to be handled by the reader
 * @param[in]   ids Motors to be handled by the reader
 * @param[in]   portHandler Object handling port communication
 * @param[in]   packetHandler Object handling packets
 * @param[in]   hal Previouly initialized Hal object
 * @param[in]   forceIndirect Boolean: 1 to force the reader to be indirect address
 *              (has no effect if at least 2 fields)
 */
Reader::Reader(vector<Fields> list_fields, vector<int> ids, dynamixel::PortHandler *portHandler,
                            dynamixel::PacketHandler *packetHandler, Hal hal, bool forceIndirect)
{
    portHandler_ = portHandler;
    packetHandler_ = packetHandler;
    m_hal = hal;
    m_ids = ids;

    m_list_fields = list_fields;

    getDataByteSize();

    if (list_fields.size() == 1 && !forceIndirect) {
        m_isIndirectHandler = false;
        checkMotorCompatibility(list_fields[0]);
    }

    else {
        m_isIndirectHandler = true;
        checkMotorCompatibility(INDIR_DATA_1);
        setIndirectAddresses();
    }

    m_groupSyncReader = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, m_data_address, m_data_byte_size);

    // Create the table to save read data
    m_dataFromMotor = new float *[m_ids.size()]; 
    for (int i=0; i<m_ids.size(); i++)
        m_dataFromMotor[i] = new float[m_list_fields.size()];

    cout << "Dxl reader created!" << endl;

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
    m_groupSyncReader->clearParam();
}

/**
 * @brief       Add a motor to the list of motors who will read
 * @param[in]   id ID of the motor
 * @retval      bool: true if motor added successfully
 */
bool Reader::addParam(uint8_t id)
{
    bool dxl_addparam_result = m_groupSyncReader->addParam(id);
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

    clearParam();    

    // Add the input motors to the reading list
    for (int i=0; i<ids.size(); i++){
        dxl_addparam_result = addParam(ids[i]);
        if (dxl_addparam_result != true) {
            cout << "Adding parameters failed for ID = " << ids[i] << endl;
            exit(1);
        }
    }

    // Read the motors' sensors
    dxl_comm_result = m_groupSyncReader->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        cout << packetHandler_->getTxRxResult(dxl_comm_result) << endl;
        exit(1);
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
    Fields field;
    int field_idx = 0;
    int field_length = 0;
    uint8_t offset = 0;

    for (int i=0; i<ids.size(); i++){
        for (int j=0; j<m_list_fields.size(); j++){
            field = m_list_fields[j];
            getFieldPosition(field, field_idx, field_length);

            dxl_getdata_result = m_groupSyncReader->isAvailable(ids[i], m_data_address + offset, field_length);

            if (dxl_getdata_result != true)
            {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed \n", ids[i]);
                exit(1);
            }

            offset += field_length;
        }

        offset = 0;
    }
}


/**
 * @brief       The reading being successful, save the read data into the output matrix
 * @param[in]   ids List of motors whose fields have been successfully read
 * @todo Change the goto to a cleaner way
 * @retval      void
 */
void Reader::populateOutputMatrix(vector<int> ids)
{
    Fields field;
    int field_idx = 0;
    int field_length = 0;
    uint8_t offset = 0;
    int32_t paramData;
    float units, data;
    int id = 0, row = 0, col = 0;

    for (int i=0; i<ids.size(); i++){
        for (int j=0; j<m_list_fields.size(); j++){
            field = m_list_fields[j];
            id = ids[i];

            getFieldPosition(field, field_idx, field_length);
            units = m_hal.getControlParametersFromID(id, field).unit;

            paramData = m_groupSyncReader->getData(id, m_data_address + offset, field_length);

            // Transform data from parametrized value to SI units
            if (field != GOAL_POS && field != PRESENT_POS &&
                field != MIN_POS_LIMIT && field != MAX_POS_LIMIT &&
                field != HOMING_OFFSET) {
                data = paramData * units;        
            }
            else
                data = position2Angle(paramData, id, units);

            // Save the converted value into the output matrix
            for (row=0; row<ids.size(); row++){
                for(col=0; col<m_list_fields.size(); col++){
                    if (ids[row] == id && m_list_fields[col] == field)
                        goto fill_matrix;
                }
            }

            fill_matrix:
            m_dataFromMotor[row][col] = data;
            
            // Offset for the data address
            offset += field_length;
        }

        offset = 0;
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

    if (model == 1030 || model == 1000 || model == 311){
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