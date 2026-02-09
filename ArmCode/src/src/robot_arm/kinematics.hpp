#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "stdbool.h"
#include "stdint.h"

/**
 * @file kinematics.hpp
 * @author Mobius
 * @brief Forward and inverse kinematics solution
 * @version 1.0
 * @date 2024-12-13
 *
 * @copyright Copyright (c) 2024 Hiwonder
 *
 */
 
 #define PI 3.1415926f
 
// Link numbering from bottom to top, unit: cm
#define LINKAGE_1    				 2.89f
#define LINKAGE_2					 10.43f
#define LINKAGE_3		 				8.9f
#define LINKAGE_4	 					17.7f

#define MIN_KNOT6_ANGLE							-90.0f
#define MAX_KNOT6_ANGLE							 90.0f
#define MIN_KNOT5_ANGLE							  0.0f
#define MAX_KNOT5_ANGLE							180.0f
#define MIN_KNOT4_ANGLE							-90.0f
#define MAX_KNOT4_ANGLE							 90.0f
#define MIN_KNOT3_ANGLE							-90.0f
#define MAX_KNOT3_ANGLE							 90.0f


typedef enum 
{
	K_OK = 1,
	INVAILD
}KinematicsStatusTypedef;

typedef struct
{
	float x;
	float y;
	float z;
}VectorObjectTypeDef;

typedef struct
{
	float rad;
	float theta;
}KnotObjectTypeDef;

typedef struct KinematicsObject KinematicsObjectTypeDef;
struct KinematicsObject
{
	float alpha;
	VectorObjectTypeDef vector;
	KnotObjectTypeDef knot[4];
};

/**
 * @brief Convert radians to degrees
 * 
 * @param  rad
 * @return angle in degrees
 */
float rad2theta(float rad);

/**
 * @brief Convert degrees to radians
 * 
 * @param  theta
 * @return radians
 */
float theta2rad(float theta);

/**
 * @brief Initialize kinematics
 * 
 * @param  self		pointer to object needing control
 * @return NULL 
 */
void kinematics_init(KinematicsObjectTypeDef* self);

/**
 * @brief Inverse kinematics solution
 * 
 * @param  self		pointer to object needing control
 * @return  K_OK		has solution
 * 			INVAILD	no solution
 */
uint8_t ikine(KinematicsObjectTypeDef* self);

/**
 * @brief Forward kinematics solution
 * 
 * @param  knot0_theta	joint 1 angle from bottom to top
 * @param  knot1_theta	joint 2 angle from bottom to top
 * @param  knot2_theta	joint 3 angle from bottom to top
 * @param  knot3_theta	joint 4 angle from bottom to top
 * @return VectorObjectTypeDef structure type
 */
VectorObjectTypeDef fkine(float knot0_theta, float knot1_theta, float knot2_theta, float knot3_theta);

#endif
