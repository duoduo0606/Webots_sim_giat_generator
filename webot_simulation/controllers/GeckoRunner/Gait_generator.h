//
// Created by Yuhang Xing on 10/9/20.
//

#ifndef GAIT_GENERATOR_GAIT_GENERATOR_H
#define GAIT_GENERATOR_GAIT_GENERATOR_H

#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#define L1 70
#define L2 50
#define L3 24

#define B_L 360
#define B_W 80


using namespace std;

class Gait_generator
{
public:
    Gait_generator();
    //-----Structure type declaration-----

    struct xyz_position
    {
        float px;
        float py;
        float pz;
    };

    struct gait_para
    {
        float step_length;
        float step_width;
        float step_depth;
        float yaw_angle;
    };

    struct time_para
    {
        float t1;//end of 1st supporting segment
        float t2;//start of 2nd supporting segment
    };

    //-----variable declaration-----
    float full_cycle_time,time_step;
    float t;
    gait_para gait_para_var;//{step_length,step_width,step_depth,yaw_angle}
    float rf_leg_pos[4],rb_leg_pos[4],lf_leg_pos[4],lb_leg_pos[4];
    float rf_shoulder_pos[4],rb_shoulder_pos[4],lf_shoulder_pos[4],lb_shoulder_pos[4];
    time_para rf_time_para,rb_time_para,lf_time_para,lb_time_para;

    //-----Function declaration-----
    vector< vector<float> > runner(float t);
    xyz_position target_position_generator(float leg_pos[]);
    vector<float> motor_theta_generator(xyz_position target_position,Gait_generator::time_para leg_time_para,float leg_pos[],float shoulder_pos[]);
    vector<float> DH_inversekinematic(xyz_position delta_position);


};

#endif //GAIT_GENERATOR_GAIT_GENERATOR_H
