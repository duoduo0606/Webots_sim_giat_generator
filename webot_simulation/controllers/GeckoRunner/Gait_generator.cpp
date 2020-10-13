//
// Created by Yuhang Xing on 10/9/20.
//

#include "Gait_generator.h"

using namespace std;

Gait_generator::Gait_generator()
{
    full_cycle_time = 8;
    time_step = 0.01;
    
    gait_para_var.step_length = 20;
    gait_para_var.step_width = -10;
    gait_para_var.step_depth = 0;
    gait_para_var.yaw_angle = -0.15;
 

    rf_leg_pos[0] = B_L/2;
    rf_leg_pos[1] = -(B_W/2+L1);
    rf_leg_pos[2] = -L3;
    rf_leg_pos[3] = 1;

    rb_leg_pos[0] = -B_L/2;
    rb_leg_pos[1] = -(B_W/2+L1);
    rb_leg_pos[2] = -L3;
    rb_leg_pos[3] = 1;

    lf_leg_pos[0] = B_L/2;
    lf_leg_pos[1] = B_W/2+L1;
    lf_leg_pos[2] = -L3;
    lf_leg_pos[3] = 1;

    lb_leg_pos[0] = -B_L/2;
    lb_leg_pos[1] = B_W/2+L1;
    lb_leg_pos[2] = -L3;
    lb_leg_pos[3] = 1;

    rf_shoulder_pos[0] = B_L/2-L2;
    rf_shoulder_pos[1] = -B_W/2;
    rf_shoulder_pos[2] = 0;
    rf_shoulder_pos[3] = 1;

    rb_shoulder_pos[0] = -B_L/2-L2;
    rb_shoulder_pos[1] = -B_W/2;
    rb_shoulder_pos[2] = 0;
    rb_shoulder_pos[3] = 1;

    lf_shoulder_pos[0] = B_L/2-L2;
    lf_shoulder_pos[1] = B_W/2;
    lf_shoulder_pos[2] = 0;
    lf_shoulder_pos[3] = 1;

    lb_shoulder_pos[0] = -B_L/2-L2;
    lb_shoulder_pos[1] = B_W/2;
    lb_shoulder_pos[2] = 0;
    lb_shoulder_pos[3] = 1;

    rf_time_para.t1 = 0;
    rf_time_para.t2 = 2;
    
    rb_time_para.t1 = 2;
    rb_time_para.t2 = 4;
    
    lf_time_para.t1 = 4;
    lf_time_para.t2 = 6;
    
    lb_time_para.t1 = 6;
    lb_time_para.t2 = 8;
    
    t = 0;
    
}

vector< vector<float> > Gait_generator::runner(float t)
{
    vector<float> rf_theta;
    vector<float> rb_theta;
    vector<float> lf_theta;
    vector<float> lb_theta;

    vector< vector<float> > leg_theta;

    xyz_position rf_target_position;
    xyz_position rb_target_position;
    xyz_position lf_target_position;
    xyz_position lb_target_position;

    //int i,n;
    int k,p;

    //n = int(full_cycle_time/time_step);
/*
    for (i = 0; i < n; i++)
    {
        t = i*time_step;
        cout << "t=" << t << endl;
*/

        rf_target_position = target_position_generator(rf_leg_pos);
        rf_theta = motor_theta_generator(rf_target_position,rf_time_para,rf_leg_pos,rf_shoulder_pos);

        rb_target_position = target_position_generator(rb_leg_pos);
        rb_theta = motor_theta_generator(rb_target_position,rb_time_para,rb_leg_pos,rb_shoulder_pos);
        rb_theta[0] = rb_theta[0];
        rb_theta[1] = rb_theta[1];
        rb_theta[2] = rb_theta[2];
        
        lf_target_position = target_position_generator(lf_leg_pos);
        lf_theta = motor_theta_generator(lf_target_position,lf_time_para,lf_leg_pos,lf_shoulder_pos);
        lf_theta[0] = -lf_theta[0];
        lf_theta[1] = -lf_theta[1];
        lf_theta[2] = -lf_theta[2];
        //cout << lf_theta[0] << endl;
        
        lb_target_position = target_position_generator(lb_leg_pos);
        lb_theta = motor_theta_generator(lb_target_position,lb_time_para,lb_leg_pos,lb_shoulder_pos);
        lb_theta[0] = -lb_theta[0];
        lb_theta[1] = -lb_theta[1];
        lb_theta[2] = -lb_theta[2];
        
        leg_theta.clear();
        leg_theta.push_back(rf_theta);
        leg_theta.push_back(rb_theta);
        leg_theta.push_back(lf_theta);
        leg_theta.push_back(lb_theta);

        for (k = 0; k < 4; k++)
        {
            for (p = 0; p < 3; p++)
            {
                cout << setw(10) << leg_theta[k][p] << "\t";
            }
            cout << endl;
        } //end k,p
    //} //end i
    return leg_theta;
}//end runner

Gait_generator::xyz_position Gait_generator::target_position_generator(float leg_pos[])
{
    float tran_matrix[4][4];
    float p[4];
    int i,j,k;
    xyz_position target_position;

    for (k = 0; k < 4; k++)
    {
        p[k] = 0;
    }

    tran_matrix[0][0] = cos(gait_para_var.yaw_angle);
    tran_matrix[1][0] = sin(gait_para_var.yaw_angle);
    tran_matrix[2][0] = 0;
    tran_matrix[3][0] = 0;
    tran_matrix[0][1] = -sin(gait_para_var.yaw_angle);
    tran_matrix[1][1] = cos(gait_para_var.yaw_angle);
    tran_matrix[2][1] = 0;
    tran_matrix[3][1] = 0;
    tran_matrix[0][2] = 0;
    tran_matrix[1][2] = 0;
    tran_matrix[2][2] = 1;
    tran_matrix[3][2] = 0;
    tran_matrix[0][3] = gait_para_var.step_length;
    tran_matrix[1][3] = gait_para_var.step_width;
    tran_matrix[2][3] = gait_para_var.step_depth;
    tran_matrix[3][3] = 1;

    for (i = 0; i < 4; i++){
        for (j = 0; j < 4; j++){
              p[i] = p[i] + tran_matrix[i][j]*leg_pos[j];
        }
    }

    target_position.px = p[0] - leg_pos[0];
    target_position.py = p[1] - leg_pos[1];
    target_position.pz = p[2] - leg_pos[2];

    return target_position;
}


vector<float> Gait_generator::motor_theta_generator(xyz_position target_position,time_para leg_time_para,float leg_pos[],float shoulder_pos[])
{
    float sup_vel_x,lift_vel_x,sup_vel_y,lift_vel_y;
    float step_hight = 30;
    float del_t;
    vector<float> theta;
    xyz_position time_t_position;

    time_t_position.px = 0;
    time_t_position.py = 0;
    time_t_position.pz = 0;
    
    del_t = leg_time_para.t2-leg_time_para.t1;

    sup_vel_x = target_position.px/(full_cycle_time-del_t);
    lift_vel_x = target_position.px/del_t;
    sup_vel_y = target_position.py/(full_cycle_time-del_t);
    lift_vel_y = target_position.py/del_t;

    if ((t >= 0) && (t < leg_time_para.t1))
    {
        time_t_position.px = -sup_vel_x*t;
        time_t_position.py = -sup_vel_y*t;
        time_t_position.pz = 0;
    }

    if ((t >= leg_time_para.t1) && (t < leg_time_para.t2))
    {
        time_t_position.px = -sup_vel_x*leg_time_para.t1+lift_vel_x*(t-leg_time_para.t1);
        time_t_position.py = -sup_vel_y*leg_time_para.t1+lift_vel_y*(t-leg_time_para.t1);
        time_t_position.pz = -4*step_hight/(del_t*del_t)*(t-leg_time_para.t1)*(t-leg_time_para.t2);
    }

    if ((t >= leg_time_para.t2) && (t < full_cycle_time))
    {
        time_t_position.px = -sup_vel_x*leg_time_para.t1+target_position.px-sup_vel_x*(t-leg_time_para.t2);
        time_t_position.py = -sup_vel_y*leg_time_para.t1+target_position.py-sup_vel_y*(t-leg_time_para.t2);
        time_t_position.pz = 0;
    }

    time_t_position.px += leg_pos[0];
    time_t_position.py += leg_pos[1];
    time_t_position.pz += leg_pos[2];

    time_t_position.px -= shoulder_pos[0];
    time_t_position.py -= shoulder_pos[1];
    time_t_position.pz -= shoulder_pos[2];
    
    if ((leg_pos == lf_leg_pos) || (leg_pos == lb_leg_pos))
    {
        time_t_position.py =  -time_t_position.py ;
    }
    
    theta = DH_inversekinematic(time_t_position);

    return theta;
}

vector<float> Gait_generator::DH_inversekinematic(xyz_position time_t_pos)
{
    vector<float> theta;
    float px,py,pz;
    float theta1,theta2,theta3;
    //int a;
    px = 0;
    py = 0;
    pz = 0;
    px = -time_t_pos.py;
    py = time_t_pos.px;
    pz = time_t_pos.pz;
    /*
    theta1 = atan2(-px,py) - atan2(sqrt(px*px+py*py-L3*L3),L3);
    m = px*cos(theta1) + py*sin(theta1);
    theta3 = acos((m*m+pz*pz-L1*L1-L2*L2)/(2*L1*L2));
    theta2 = atan2(pz*(L1+L2*cos(theta3))-L2*sin(theta3),m*(L1+L2*cos(theta3))+pz*L2*sin(theta3));
    */
/*
    theta1 = atan2(L3,sqrt((px-l0)*(px-l0)+py*py-L3*L3))+atan2(py,(px-l0));
    m = ((pow((pz-l0),2)+pow((cos(theta1)*px+sin(theta1)*py-l0*cos(theta1)),2)-L1*L1-L2*L2-L3*L3)/(2*L1));
    theta3 = atan2(L2,L3)+atan2(m,sqrt(L2*L2+L3*L3-m*m));
    a = L2*cos(theta3)-L3*sin(theta3)+L1;
    b = L2*sin(theta3)+L3*cos(theta3);
    theta2 = atan2((pz-l0),sqrt(a*a+b*b-(pz-l0)*(pz-l0)))-atan2(b,a);
*/

    theta1 = asin(-L3/sqrt(px*px+pz*pz))-atan2(pz,px);
    theta3 = asin((L1*L1+L2*L2+L3*L3-px*px-py*py-pz*pz)/(2*L1*L2));
    theta2 = asin((px*px+py*py+pz*pz+L1*L1-L2*L2-L3*L3)/(2*L1*sqrt(px*px+py*py+pz*pz-L3*L3)))-atan2(sqrt(px*px+pz*pz-L3*L3),py);

    theta.clear();
    theta.push_back(theta1);
    theta.push_back(theta2);
    theta.push_back(theta3);


    return theta;
}