#ifndef __PC_BLE_CTL_HPP_
#define __PC_BLE_CTL_HPP_

#include "./../../Robot_arm.hpp"

#define LEARM_VERSION 0x01

#define APP_PACKET_HEADER                  0x55  // Communication protocol frame header
#define APP_TX_DATA_LENGTH                   2  // Send side data length excluding frame header
#define MAX_PACKET_LENGTH					64

typedef enum
{
	PACKET_HEADER_1 = 0,
	PACKET_HEADER_2,
	PACKET_DATA_LENGTH,
	PACKET_FUNCTION,
	PACKET_DATA
}PacketAnalysisStatus;

typedef enum
{
	APP_VERSION_QUERY = 1, // Firmware query
	APP_SERVO_OFFSET_READ, // Servo offset reading
	APP_MULT_SERVO_MOVE, // Control single/multiple servo pulse width
	APP_COORDINATE_SET, // Coordinate control

	APP_ACTION_GROUP_RUN = 6, // Action group run
	APP_FULL_ACTION_STOP, // Stop running action group
	APP_FULL_ACTION_ERASE, // Erase downloaded action group on control board
	APP_CHASSIS_CONTROL, // Chassis control
	APP_SERVO_OFFSET_SET, // Servo offset setting
	APP_SERVO_OFFSET_DOWNLOAD, // Servo offset download
	APP_SERVOS_RESET, // Reset pose
	APP_SERVOS_READ, // Read servo angle
	
	APP_ACTION_DOWNLOAD = 25, // Action group download
	APP_FUNC_NULL
}AppFunctionStatus;


#pragma pack(1)
typedef struct
{
	uint8_t action_frame_sum;
	uint8_t action_frame_index;
	uint8_t action_group_index;
	uint16_t running_times;
	
	uint8_t packet_header[2];
	uint8_t data_len;
	uint8_t cmd;
	uint8_t buffer[MAX_PACKET_LENGTH - 4];
}PacketObjectTypeDef;
#pragma pack()

class PC_BLE_CTL
{
private:
	uint8_t set_id;
	uint8_t servos_count;
	uint16_t set_duty;
	uint16_t running_time;

	bool unpack_successful;
    PacketObjectTypeDef tmp_packet;
    PacketObjectTypeDef packet;
	PacketAnalysisStatus packet_status;
	AppFunctionStatus status;

	struct{
		float x;
		float y;
		float z;
		int8_t pitch;
	}pose;

	void unpack(void);
	void packet_transmit(uint8_t* data, uint8_t len);
	uint8_t transmit_data(const uint8_t* pdata, uint16_t size);


public:
	void init(int pc_ble_flag);
	void PC_BLE_Task(LeArm_t* robot,Led_t* led,Buzzer_t* buzzer);
};


#endif //__PC_BLE_CTL_HPP_
