/**
 ******************************************************************************
 * @file            KMR_dxlP1_handler.cpp
 * @brief           Defines the Handler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT 
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#include "KMR_dxlP1_handler.hpp"
#include <algorithm>
#include <cstdint>

#define INDIR_OFFSET                2
#define PARAM_OFFSET                1
#define POS_DATA_SIZE               4


using std::cout;
using std::endl;
using std::vector;


namespace KMR::dxlP1
{


/*
 *****************************************************************************
 *                                Initializations
 ****************************************************************************/


/**
 * @brief       Check if the motors are compatible for a given field: same address for data storing
 * @param[in]   field Control field that the handler is taking care of
 * @retval      void
 */
void Handler::checkMotorCompatibility(Fields field)
{
    uint8_t address = -1;
    uint8_t address_prev = -1;
    int id = -1, id_prev = -1;
  
    for(int i=1; i<m_ids.size(); i++){
        id = m_ids[i];
        id_prev = m_ids[i-1];
        address = m_hal.getControlParametersFromID(id, field).address;
        address_prev = m_hal.getControlParametersFromID(id_prev, field).address;

        if(address != address_prev){
            cout << "Motors " << id << " and " << id_prev << " have incompatible addresses!" << endl;
            exit(1);
        }
    }                                                                                                                                                                                                                                                                                                                        

    if (m_ids.size() == 1)
        address = m_hal.getControlParametersFromID(m_ids[0], field).address;

    m_data_address = address;
}


/**
 * @brief       Calculate and store the byte length of data read/written by the handler. \n 
 *              Also check if the motors are field-compatible (same data lengths required for a given field)
 * @retval      void
 */
void Handler::getDataByteSize()
{
    uint8_t length = 0, length_prev = 0;
    Fields field = m_field;
    
    for (int j=1; j<m_ids.size(); j++){
        length = m_hal.getControlParametersFromID(m_ids[j], field).length;
        length_prev = m_hal.getControlParametersFromID(m_ids[j-1], field).length;       

        if(length != length_prev){
            cout << "Motors " << m_ids[j] << " and " << m_ids[j-1] << " have incompatible field lengths!" << endl;
            exit(1);
        }
    }

    if (m_ids.size() == 1)
        length = m_hal.getControlParametersFromID(m_ids[0], field).length;

    m_data_byte_size += length;

}

/*
 *****************************************************************************
 *                        Security checking functions
 ****************************************************************************/

/**
 * @brief       Check if query motors are handled by this specific handler
 * @param[in]   ids List of query motors
 * @retval      void
 */
void Handler::checkIDvalidity(vector<int> ids)
{
    for(int i=0; i<ids.size(); i++){
        if ( find(m_ids.begin(), m_ids.end(), ids[i]) == m_ids.end() ) {
            cout << "Error: motor " << ids[i] << " is not handled by this handler!" << endl;  
            exit(1);
        }
    }
}

/**
 * @brief       Check if query field is handled by this specific handler
 * @param[in]   field Query control field
 * @retval      void
 */
void Handler::checkFieldValidity(Fields field)
{
    if (field != m_field) {
        cout << "Error: field " << field << " is not handled by this handler!" << endl;  
        exit(1);
    }
}


/**
 * @brief       Get the index of a motor in the list of handled motors
 * @param[in]   id Query motor
 * @retval      Index of the motor in the list of handled motors
 */
int Handler::getMotorIndexFromID(int id)
{
    int idx = 0;

    while(m_ids[idx] != id)
        idx++;

    return idx; 
}

}
