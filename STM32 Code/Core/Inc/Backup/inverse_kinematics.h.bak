/*
 * inverse_kinematics.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gurpreetmukker
 */

#ifndef INC_INVERSE_KINEMATICS_H_
#define INC_INVERSE_KINEMATICS_H_



#endif /* INC_INVERSE_KINEMATICS_H_ */

#define LINK_0_LEN 9.1
#define LINK_1_LEN 10.45
#define LINK_2_LEN 14.64
#define LINK_3_LEN 18.5

#define BASE_MOTOR_OFFSET -9
#define SHOULDER_MOTOR_OFFSET +15
#define ELBOW_MOTOR_OFFSET 0
#define WRIST_MOTOR_OFFSET +5

#define BASE_MOTOR_MIN 0
#define SHOULDER_MOTOR_MIN 0
#define ELBOW_MOTOR_MIN 45
#define WRIST_MOTOR_MIN 30

#define SHOULDER_MOTOR_OFFSET_RAD (SHOULDER_MOTOR_OFFSET * M_PI / 180)
#define ELBOW_MOTOR_OFFSET_RAD (ELBOW_MOTOR_OFFSET * M_PI / 180)
#define WRIST_MOTOR_OFFSET_RAD (WRIST_MOTOR_OFFSET * M_PI / 180)
#define BASE_MOTOR_OFFSET_RAD (BASE_MOTOR_OFFSET * M_PI / 180)

#define BASE_MOTOR_MIN_RAD (BASE_MOTOR_MIN * M_PI / 180)
#define SHOULDER_MOTOR_MIN_RAD (SHOULDER_MOTOR_MIN * M_PI / 180)
#define ELBOW_MOTOR_MIN_RAD (ELBOW_MOTOR_MIN * M_PI / 180)
#define WRIST_MOTOR_MIN_RAD (WRIST_MOTOR_MIN * M_PI / 180)




int check_angle_validty(double alpha, double beta, double zeta, double gamma);

void calculate_trajectory(double *p0, double *pf, double v0, double vf, double tf, int steps,double* trajectory);

double norm(double vec[], int size);

void calculate_angles(double vec[], double *alpha, double *beta, double *zeta, double *gamma);

void calc_link1_vec(double alpha, double gamma, double link1, double vec[]);

void calc_link2_vec(double alpha, double beta, double gamma, double link2, double vec[]);
