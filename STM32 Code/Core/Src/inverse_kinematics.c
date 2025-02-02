/*
 * inverse_kinematics.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gurpreetmukker
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "inverse_kinematics.h"



double norm(double vec[], int size) {
    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += vec[i] * vec[i];
    }
    return sqrt(sum);
}

void calculate_angles(double vec[], double *alpha, double *beta, double *zeta, double *gamma) {
    double link0_vec[3] = {0, 0, LINK_0_LEN};
    double link3_vec[3] = {0, 0, -LINK_3_LEN};
    double r_dd_vec[3] = {vec[0] - link0_vec[0] - link3_vec[0], vec[1] - link0_vec[1] - link3_vec[1], vec[2] - link0_vec[2] - link3_vec[2]};
    double z_vec[3] = {vec[0] - link0_vec[0], vec[1] - link0_vec[1], vec[2] - link0_vec[2]};
    double z = norm(z_vec, 3);
    double zz = z*z;
    double r = norm(vec, 3);
    double r_sq = r*r;
    double link_sum = LINK_1_LEN + LINK_2_LEN;
    double link_sum_sq = link_sum*link_sum;
    double link3_len_sq = LINK_3_LEN*LINK_3_LEN;

    if (z > link_sum + LINK_3_LEN) {
        printf_("Error: Vector length is greater than the sum of the links\n");
        *alpha = 0;
        *beta = 0;
        *zeta = 0;
        *gamma = 0;
        return;
    }

    double r_dd = norm(r_dd_vec, 3);
    double r_dd_sq = r_dd*r_dd;

    if (r_dd > link_sum) {
        double alpha_d = acos((link_sum_sq + zz - link3_len_sq) / (2*link_sum*z));
        *alpha = acos((zz + LINK_0_LEN*LINK_0_LEN - r_sq) / (2*z*LINK_0_LEN)) + alpha_d;
        *beta = M_PI;
        *zeta = acos((link_sum_sq + link3_len_sq - zz) / (2*link_sum*LINK_3_LEN));
        *gamma = atan2(vec[1], vec[0]);
        *alpha = *alpha - M_PI_2;
    } else {
        double alpha_d = acos((r_dd_sq + zz - link3_len_sq) / (2*z*r_dd));
        *alpha = acos((zz + LINK_0_LEN*LINK_0_LEN - r_sq) / (2*z*LINK_0_LEN)) + alpha_d;
        *beta = acos((LINK_1_LEN*LINK_1_LEN + LINK_2_LEN*LINK_2_LEN - r_dd_sq) / (2*LINK_1_LEN*LINK_2_LEN));
        *zeta = 3*M_PI - *alpha - *beta - M_PI;
        *gamma = atan2(vec[1], vec[0]);
        *alpha = *alpha - M_PI_2;
    }
}


int check_angle_validty(double alpha, double beta, double zeta, double gamma){


    if (!((alpha >= (SHOULDER_MOTOR_OFFSET_RAD + SHOULDER_MOTOR_MIN_RAD)) && (alpha <= (M_PI + SHOULDER_MOTOR_OFFSET_RAD)))) {
        return 1;
    }
    if (!((beta >= (ELBOW_MOTOR_OFFSET_RAD + ELBOW_MOTOR_MIN_RAD)) && (beta <= (M_PI + ELBOW_MOTOR_OFFSET_RAD)))) {
        return 1;
    }
    if (!((zeta >= (WRIST_MOTOR_OFFSET_RAD + WRIST_MOTOR_MIN_RAD)) && (zeta <= (M_PI + WRIST_MOTOR_OFFSET_RAD)))) {
        return 1;
    }
    if (!((gamma >= (BASE_MOTOR_OFFSET_RAD + BASE_MOTOR_MIN_RAD)) && (gamma <= (M_PI + BASE_MOTOR_OFFSET_RAD)))) {
        return 1;
    }
    return 0;
}


void calculate_trajectory(double *pi, double *pf, double vi, double vf, double tf, int steps, double* trajectory) {
    double a_coef[3];
    double b_coef[3];
    double c_coef[3];
    double d_coef[3];
    double t = 0;
    double t_inc = tf/steps;

    for (int i = 0; i < 3; i++) {
        a_coef[i] = (2*pi[i] - 2*pf[i] + tf*vi + tf*vf) / pow(tf, 3);
        b_coef[i] = (-3*pi[i] + 3*pf[i] - 2*tf*vi - tf*vf) / pow(tf, 2);
        c_coef[i] = vi;
        d_coef[i] = pi[i];
    }

    for(int j = 0; j <= steps; j++) {
        double p[3] = {a_coef[0]*pow(t, 3) + b_coef[0]*pow(t, 2) + c_coef[0]*t + d_coef[0], a_coef[1]*pow(t, 3) + b_coef[1]*pow(t, 2) + c_coef[1]*t + d_coef[1], a_coef[2]*pow(t, 3) + b_coef[2]*pow(t, 2) + c_coef[2]*t + d_coef[2]};
        //TRAJECTORY SHAPE - NUM_OF_MOTORS x
        calculate_angles(p, &trajectory[0*(steps+1)+j], &trajectory[1*(steps+1)+j], &trajectory[2*(steps+1)+j], &trajectory[3*(steps+1)+j]);
        t += t_inc;
    }
}




//void calc_link1_vec(double alpha, double gamma, double link1, double vec[]) {
//    double x_y = link1 * cos(alpha);
//    vec[0] = x_y * cos(gamma);
//    vec[1] = x_y * sin(gamma);
//    vec[2] = link1 * sin(alpha);
//}
//
//void calc_link2_vec(double alpha, double beta, double gamma, double link2, double vec[]) {
//    double x_y = link2 * cos(M_PI - alpha - beta);
//    vec[2] = -link2 * sin(M_PI - alpha - beta);
//    vec[0] = x_y * cos(gamma);
//    vec[1] = x_y * sin(gamma);
//}
