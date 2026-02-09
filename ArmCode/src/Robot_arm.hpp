#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__
/**
 * @file robot_arm.h
 * @author Min
 * @brief Robot arm control implementation
 * @version 1.0
 * @date 2024-12-31
 *
 * @copyright Copyright (c) 2024 Hiwonder
 *
 */
#include "stdint.h"
#include "stdbool.h"
#include "Hiwonder.hpp"
#include "./src/robot_arm/Kinematics.hpp"
#include "Config.h"

#define SERVO_NUM               6

// Software version
#define SOFTWARE_VERSION        1

#define DEFAULT_X						  0.0f
#define DEFAULT_Y						  0.0f
#define DEFAULT_Z						  0.0f
#define DEFAULT_CLAW_OPEN_ANGLE			 90.0f
#define DEFAULT_CLAW_ROTATION_ANGLE		 90.0f

#define MIN_OPEN_ANGLE					  0.0f
#define MAX_OPEN_ANGLE					 90.0f
#define MIN_ROTATION_ANGLE			 	-90.0f
#define MAX_ROTATION_ANGLE				 90.0f

#define MIN_PITCH						-90.0f
#define MAX_PITCH						 90.0f

// Flash data storage start base addresses
#define LOGO_BASE_ADDRESS          		   0UL  // This base address is used to store identification LOGO
#define SERVOS_OFFSET_BASE_ADDRESS				 4096UL	// Servo offset saving base address
#define ACTION_FRAME_SUM_BASE_ADDRESS 	 8192UL  // This base address is used to store the number of actions in each action group
#define ACTION_GROUP_BASE_ADDRESS 		 12288UL	// This base address is used to store downloaded action group files

#define ACTION_FRAME_SIZE					21  // One action frame occupies 32 bytes 
#define ACTION_GROUP_SIZE			 	  8192  // 1 action group has 8KB memory space
#define ACTION_GROUP_MAX_NUM               255  // Default maximum 255 action groups storage

// Get low 8 bits of A
#define GET_LOW_BYTE(A) ((uint8_t)(A))
// Get high 8 bits of A
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
// Merge high and low 8 bits into 16 bits
#define MERGE_HL(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define DEFAULT_X						 15.0f
#define DEFAULT_Y						  0.0f
#define DEFAULT_Z						  2.0f

typedef enum
{
	ACTION_FRAME_START = 0,
	ACTION_FRAME_RUNNING,
	ACTION_FRAME_IDLE
}ActionFrameStatusTypeDef;

typedef enum
{
	ACTION_GROUP_START = 0,
	ACTION_GROUP_RUNNING,
	ACTION_GROUP_END_PERIOD,
	ACTION_GROUP_IDLE
}ActionGroupStatusTypeDef;

typedef struct
{
	uint8_t 			 index;			/* Current action frame index */
	uint32_t 		 time; 			/* Running time of current action frame */
	uint8_t		 status; 		/* Current running flag */
}ActionFrameHandleTypeDef;

typedef struct
{
	uint8_t 		 index; 			/* Current action group index */
	uint8_t		 sum; 			/* Total frames in current action group */
	uint8_t		 running_times; 		/* Running times for current action group */
	uint8_t		 status; 		/* Current running flag */
	uint32_t	 time; 			/* Running time of current action group */

	ActionFrameHandleTypeDef frame;
	
}ActionGroupHandleTypeDef;

typedef struct
{
	uint8_t 				 cmd;
	ActionGroupHandleTypeDef action_group;
}RobotArmHandleTypeDef;



class LeArm_t{
    public:
        void init(void);
        void reset(uint32_t time = 800);
        /**
        * @brief Robot arm coordinate control interface
        * 
        * @param  target_x 	Target x coordinate
        * @param  target_y		Target y coordinate
        * @param  target_z		Target z coordinate
        * @param  pitch		Target pitch angle
        * @param  min_pitch	Minimum pitch angle
        * @param  max_pitch	Maximum pitch angle
        * @param  time			Running time
        * @return true			Solvable
        * 		   false		Unsolvable
        */
        uint8_t coordinate_set(float target_x,float target_y,float target_z,float pitch,float min_pitch,float max_pitch,uint32_t time);

        /* Single joint control interface */
        /* angle range [0, 90] */
        void claw_set(float open_angle, uint32_t open_angle_time);
        /* angle range [-90, 90] */
        void roll_set(float rotation_angle, uint32_t rotation_angle_time);
        /* id range [1, 6] from top to bottom */
        void knot_run(uint8_t id, int target_duty, uint32_t time);
        /* id range [1, 6] from top to bottom */
        void knot_stop(uint8_t id);
        /* Read servo position (for PWM servos) */
        uint16_t knot_read(uint8_t id);

// #if (SERVO_TYPE == TYPE_PWM_SERVO)
        /* Check whether servo has finished movement (bus servos cannot use this function) */
        bool knot_finish(uint8_t id);
// #else
        void serial_servo_offset_init(void);
// #endif

        /* Action group control interface */
        void action_group_reset(void);
        void action_group_stop(void);
        void action_group_erase(void);
        bool action_group_run(uint8_t action_group_index, uint8_t repeat_times);

        void action_run(uint8_t action_group_index, uint8_t repeat_times);

        /**
        * @brief Action group data write interface
        * 
        * @param  self
        * @param  action_group_index 	Action group index
        * @param  frame_num			Total frames in the action group
        * @param  frame_index		Index of frame being written, range [0,255]
        * @param  pdata				Pointer to frame data
        * @param  size				Frame data length
        */
        int action_group_save(uint8_t action_group_number, uint8_t frame_num,uint8_t frame_index,uint8_t* pdata,uint16_t size);

        /* Offset setting interface */
        int8_t offset_read(uint8_t id);
        void offset_set(uint8_t id, int8_t value);
        void offset_save(void);


        uint8_t get_servo_type(void);
        
    private:
        Flash_ctl_t flash_obj;
// #if (SERVO_TYPE == TYPE_PWM_SERVO)
        PwmServo_t  pwmservo_obj;
// #else
        BusServo_t  busservo_obj;
        int8_t      bus_servo_offset[6];
// #endif
        KinematicsObjectTypeDef  kinematics;
        RobotArmHandleTypeDef robot_arm;

        uint8_t servo_type;
        uint8_t read_servo_type(void);
        uint8_t action_frame_run(uint8_t action_group_index, uint8_t frame_index);
        void theta2servo(KinematicsObjectTypeDef* self, float time);
        void action_group_init();
};

#endif
