/**
 ******************************************************************************
 * @file            KMR_dxlP1_reader.hpp
 * @brief           Header for the KMR_dxlP1_reader.cpp file.
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 08/2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 08/2023
 * @authors katarina.lichardova@km-robota.com, 08/2023
 ******************************************************************************
 */

#ifndef KMR_DXLP1_READER_HPP
#define KMR_DXLP1_READER_HPP

#include "KMR_dxlP1_handler.hpp"

namespace KMR::dxlP1
{

/**
 * @brief       Custom Reader class that contains a dynamixel::GroupSyncRead object
 * @details 	This custom Reader class simplifies greatly the creation of dynamixel reading handlers. \n 
 * 				It takes care automatically of address assignment, even for indirect address handling. 
 */
class Reader : public Handler
{
protected:
	dynamixel::GroupBulkRead *m_groupBulkReader;

	void clearParam();
	bool addParam(uint8_t id);
	void checkReadSuccessful(std::vector<int> ids);
	void populateOutputMatrix(std::vector<int> ids);
	float position2Angle(int32_t position, int id, float units);

public:
	float *m_dataFromMotor;  // Table holding the read values from motors
	int *motorIndices_dataFromMotor; // used? @todo
	int *fieldIndices_dataFromMotor; // used? @todo

	Reader(Fields field, std::vector<int> ids,
			dynamixel::PortHandler *portHandler,
			dynamixel::PacketHandler *packetHandler, Hal hal);
	~Reader();
	void syncRead(std::vector<int> ids);
};

} // namespace KMR::dxl

#endif