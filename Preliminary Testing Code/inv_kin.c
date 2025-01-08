#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define LINK_0_LEN 9.1
#define LINK_1_LEN 10.45
#define LINK_2_LEN 14.64
#define LINK_3_LEN 15

#define BASE_MIN_DC 0
#define BASE_MAX_DC 15
#define SHOULDER_MIN_DC 0
#define SHOULDER_MAX_DC 15
#define ELBOW_MIN_DC 0
#define ELBOW_MAX_DC 15
#define WRIST_MIN_DC 0
#define WRIST_MAX_DC 15
#define GRIPPER_MIN_DC 0
#define GRIPPER_MAX_DC 15

int counter = 0;

double norm(double vec[], int size) {
    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += vec[i] * vec[i];
    }
    return sqrt(sum);
}

void calculate_angles(double link0, double link1, double link2, double link3, double *vec, double *alpha, double *beta, double *zeta, double *gamma) {
    double link0_vec[3] = {0, 0, link0};
    double link3_vec[3] = {0, 0, -link3};

    double r_dd_vec[3] = {vec[0] - link0_vec[0] - link3_vec[0], vec[1] - link0_vec[1] - link3_vec[1], vec[2] - link0_vec[2] - link3_vec[2]};
    double z_vec[3] = {vec[0] - link0_vec[0], vec[1] - link0_vec[1], vec[2] - link0_vec[2]};
    double z = norm(z_vec, 3);

    if (z > link1 + link2 + link3) {
        printf("Error: Vector length is greater than the sum of the links\n");
        *alpha = 0;
        *beta = 0;
        *zeta = 0;
        *gamma = 0;
        return;
    }

    if (norm(r_dd_vec, 3) > link1 + link2) {
        double alpha_dd = 0;
        double alpha_d = acos(((link1 + link2)*(link1 + link2) + z*z - link3*link3) / (2*(link1 + link2)*z)) + alpha_dd;
        double r = norm(vec, 3);
        *alpha = acos((z*z + link0*link0 - r*r) / (2*z*link0)) + alpha_d;

        *beta = M_PI;
        *zeta = acos(((link1+link2)*(link1+link2) + link3*link3 - z*z) / (2*(link1+link2)*link3));
        *gamma = atan2(vec[1], vec[0]);
    } else {
        double r_dd = norm(r_dd_vec, 3);
        double alpha_dd = acos((link1*link1 + r_dd*r_dd - link2*link2) / (2*link1*r_dd));
        double alpha_d = acos((r_dd*r_dd + z*z - link3*link3) / (2*z*r_dd)) + alpha_dd;
        double r = norm(vec, 3);
        *alpha = acos((z*z + link0*link0 - r*r) / (2*z*link0)) + alpha_d;
        *beta = acos((link1*link1 + link2*link2 - r_dd*r_dd) / (2*link1*link2));
        *zeta = 3*M_PI - *alpha - *beta - M_PI;
        *gamma = atan2(vec[1], vec[0]);
    }
}

void calculate_trajectory(double *p0, double *pf, double v0, double vf, double tf, int steps,double* trajectory) {
    double a_coef[3];
    double b_coef[3];
    double c_coef[3];
    double d_coef[3];
    double t = 0;
    double t_inc = tf/steps;
    
    for (int i = 0; i < 3; i++) {
        a_coef[i] = (2*p0[i] - 2*pf[i] + tf*v0 + tf*vf) / pow(tf, 3);
        b_coef[i] = (-3*p0[i] + 3*pf[i] - 2*tf*v0 - tf*vf) / pow(tf, 2);
        c_coef[i] = v0;
        d_coef[i] = p0[i];
    }

    for(int j = 0; j <= steps; j++) {
        double p[3] = {a_coef[0]*pow(t, 3) + b_coef[0]*pow(t, 2) + c_coef[0]*t + d_coef[0], a_coef[1]*pow(t, 3) + b_coef[1]*pow(t, 2) + c_coef[1]*t + d_coef[1], a_coef[2]*pow(t, 3) + b_coef[2]*pow(t, 2) + c_coef[2]*t + d_coef[2]}; 
        calculate_angles(LINK_0_LEN, LINK_1_LEN, LINK_2_LEN, LINK_3_LEN, p, &trajectory[0*(steps+1)+j], &trajectory[1*(steps+1)+j], &trajectory[2*(steps+1)+j], &trajectory[3*(steps+1)+j]);
        t += t_inc;
    } 
}
void calc_link1_vec(double alpha, double gamma, double link1, double vec[]) {
    double x_y = link1 * cos(alpha - M_PI/2);
    vec[0] = x_y * cos(gamma);
    vec[1] = x_y * sin(gamma);
    vec[2] = link1 * sin(alpha - M_PI/2);
}

void calc_link2_vec(double alpha, double beta, double gamma, double link2, double vec[]) {
    double x_y = link2 * cos((1.5)*M_PI - alpha - beta);
    vec[2] = -link2 * sin((1.5)*M_PI - alpha - beta);
    vec[0] = x_y * cos(gamma);
    vec[1] = x_y * sin(gamma);
}

void increment_counter(){
    counter++;
}

int main() {
    
    double vec[3] = {0.1, 0.1 , 3.1};
    double vec2[3] = {0.1, 0.1, 3.2};
    double alpha, beta, gamma, zeta;

    calculate_angles(LINK_0_LEN, LINK_1_LEN, LINK_2_LEN, LINK_3_LEN, vec, &alpha, &beta, &zeta, &gamma);
    printf("alpha: %f, beta: %f, zeta: %f, gamma: %f\n", alpha, beta, zeta, gamma);
    double link0_vec[3] = {0, 0, LINK_0_LEN};
    printf("link0_vec: %f, %f, %f\n", link0_vec[0], link0_vec[1], link0_vec[2]);
    double link1_vec[3];
    calc_link1_vec(alpha, gamma, LINK_1_LEN, link1_vec);
    printf("link1_vec: %f, %f, %f\n", link1_vec[0], link1_vec[1], link1_vec[2]);
    double link2_vec[3];
    double link3_vec[3];
    if (beta==M_PI){
        printf("beta1: %f\n", beta);
        link2_vec[0]= link1_vec[0]/LINK_1_LEN*LINK_2_LEN;
        link2_vec[1]= link1_vec[1]/LINK_1_LEN*LINK_2_LEN;
        link2_vec[2]= link1_vec[2]/LINK_1_LEN*LINK_2_LEN;
        calc_link2_vec(alpha, zeta, gamma, LINK_3_LEN, link3_vec);
    }
    else{
        printf("beta2: %f\n", beta);
        double link2_vec[3];
        calc_link2_vec(alpha, beta, gamma, LINK_2_LEN, link2_vec);
        link3_vec[0]= 0;
        link3_vec[1]= 0;
        link3_vec[2]= -LINK_3_LEN;
    }

    printf("link2_vec: %f, %f, %f\n", link2_vec[0], link2_vec[1], link2_vec[2]);
    printf("link3_vec: %f, %f, %f\n", link3_vec[0], link3_vec[1], link3_vec[2]);
    //print the sume of link1_vec and link2_vec and link3_vec
    printf("sum: %f, %f, %f\n", link0_vec[0] + link1_vec[0] + link2_vec[0] + link3_vec[0], link0_vec[1] + link1_vec[1] + link2_vec[1] + link3_vec[1], link0_vec[2] + link1_vec[2] + link2_vec[2] + link3_vec[2]);

    int steps = 25;
    double tf = 1;

    double trajectory[4][steps+1];


    calculate_trajectory(vec, vec2, 0, 0, tf, steps, (double*) trajectory);

    for(int i = 0; i <= steps; i++) {
        // double p[3] = {trajectory[0*steps+i], trajectory[1*steps+i], trajectory[2*steps+i]};
        // calculate_angles(LINK_0_LEN, LINK_1_LEN, LINK_2_LEN, LINK_3_LEN, p, &alpha, &beta, &zeta, &gamma);
        printf("alpha: %0f, beta: %f, zeta: %f, gamma: %f\n", trajectory[0][i], trajectory[1][i],trajectory[2][i], trajectory[3][i]);
    }

    return 0;

}