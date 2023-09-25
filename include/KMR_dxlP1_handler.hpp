/**
 ******************************************************************************
 * @file            KMR_dxlP1_handler.hpp
 * @brief           Header for the KMR_dxlP1_handler.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXLP1_HANDLER_HPP
#define KMR_DXLP1_HANDLER_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "KMR_dxlP1_hal.hpp"
#include <cstdint>

namespace KMR::dxlP1
{


/**
 * @brief       Parent class, to be specialized as a Reader or Writer
 * @details		This class is not usable by itself, it is a non-specialized sketelon inherited
 * 				by the child classes Reader and Writer. \n
 * 				It contains functionalities to check the viability of sync readers/writers 
 * 				that will be defined in child classes (motor compatibility).
 */
class Handler
{
public:
	std::vector<int> m_ids;		// All IDs handled by this specific handler
	Fields m_field;				// Field handled by this specific handler


protected:
	dynamixel::PacketHandler *packetHandler_;
	dynamixel::PortHandler *portHandler_;
	Hal m_hal;
	uint8_t m_data_address = -1;		// Address where the data is written/read
	uint8_t m_data_byte_size = 0;		// Total data byte size handled by the handler	

	void checkMotorCompatibility(Fields field);
	void getDataByteSize();
	void checkIDvalidity(std::vector<int> ids);
	void checkFieldValidity(Fields field);
	int getMotorIndexFromID(int id);

	// Methods that need to be implemented in child classes
	virtual void clearParam() = 0; // Pure Virtual Function
};

} // namespace KMR::dxl

#endif