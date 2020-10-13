// File:          GeckoRunner.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include<ctime>
#include<iostream>
#include<math.h>

//#include "Motion.hpp"
#include "Gait_generator.cpp" 
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

const char* MOTION_NAMES[4][3] = {
   {"RF0", "RF1", "RF2"},
   {"RH0", "RH1", "RH2"},
   {"LF0", "LF1", "LF2"},
   {"LH0", "LH1", "LH2"}
};

const char* SENSOR_NAMES[4][3] = {
   {"RF0 sensor", "RF1 sensor", "RF2 sensor"},
   {"RH0 sensor", "RH1 sensor", "RH2 sensor"},
   {"LF0 sensor", "LF1 sensor", "LF2 sensor"},
   {"LH0 sensor", "LH1 sensor", "LH2 sensor"}
};
const char* TOUCH_SENSOR_NAMES[4] = {
"RF touch sensor", "RH touch sensor", 
"LF touch sensor", "LH touch sensor"
};

//extern float limb_force_sensor[4];
Motor *limb_motors[4][3];
TouchSensor *limb_force_sensor[4];

//virtual enable (timeStep);
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
/*  // delay
static void sleep_ms(unsigned int secs)
{
struct timeval tval;

tval.tv_sec=secs/1000;

tval.tv_usec=(secs*1000)%1000000;

select(0,NULL,NULL,NULL,&tval);
}
*/
// This fution used to delay ms in linux environment 
int main(int argc, char **argv) {
 
  Gait_generator gg;
  vector< vector<float> > theta;
  

  
    
  
  
  // create the Robot instance.
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();
  //wb_touch_sensor_enable(timeStep);
  

  for(int i=0;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      limb_motors[i][j] = robot->getMotor(MOTION_NAMES[i][j]);
    }
  }
  // get the time step of the current world.
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  
  
  
  theta = gg.runner(gg.t);
  
  if (abs(gg.t - 8) <= 5e-3)
  {
    gg.t = 0;
  }
  
  cout<<gg.t<<"starting"<<endl;
  for(int i=0;i<4;i++)
  {  
    for(int j=0;j<3;j++)
    {
      //cout<<i<<","<<j<<endl;
      limb_motors[i][j]->setPosition(theta[i][j]);
    }
    
  }
    cout<<gg.t<<"ending"<<endl;
    gg.t = gg.t + gg.time_step;
   
    
   
   
    //  motion_controller.Outputall_PWM();
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
