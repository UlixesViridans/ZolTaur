#ifndef STEPPER_ACCELERATION_CONTROLLER_H
#define STEPPER_ACCELERATION_CONTROLLER_H

#include "Motor_Position.h"

 /*
  * Acceleration controller for the stepper controller
  * This Struct takes in a max speed, a distance in decidegrees and an acceleration 
  * coefficient.
  * The struct then calculates the positions that the controller changes speed at.
  * The struct shall also calculate quarter, half, and three-quarter speed values
  * 
  * Lastly the struct has a statemachine associated with it that helps keep track of
  * acceration states and values
  *
  * -By Aut
  */

typedef enum : uint8_t
{
  A_QUARTER = 3,
  A_HALF = 2,
  A_THREE_QUARTER = 1,
  A_FULL = 0
}Speeds;

typedef enum : uint8_t
{
  S_ACCEL_1,
  S_ACCEL_2,
  S_ACCEL_3
}Acceleration_States;

typedef struct
{
  //maximum speed of motor in decidegrees
  // 0 is max speed
  // 1 is 3/4 speed
  // 2 is 1/2 speed
  // 3 is 1/4 speed
  uint16_t speeds[4];

  //speed change positions positions
  // 0: Starting Position Accel to quarter speed 
  // 1: Accel to half speed
  // 2: Accel to 3/4 speed
  // 3: Accel to full speed
  // 4: Deaccel to 3/4 speed
  // 5: Deaccel to half speed
  // 6: Deaccel to quarter speed
  // 7: End point
  MotorPosition_TMC2208 inflectionPoints[8];
  MotorPosition_TMC2208 distance;
  // Controls how aggressively the
  // Motor accelerates
  // 1 Slow acceleration
  // 2 Medium acceleration
  // 4 Faster acceleration
  uint8_t accelCoefficient = 1;
}AccelerationController;




void initAccelerationController( 
  AccelerationController * A_Controller, 
  uint16_t maxSpeed, 
  MotorPosition_TMC2208 * StartingPos, 
  MotorPosition_TMC2208 * EndingPos,
  uint8_t accelCoefficient
  )
{
  //Set the speeds
  A_Controller->speeds[A_FULL] = maxSpeed;
  A_Controller->speeds[A_THREE_QUARTER] = maxSpeed/4*3;
  A_Controller->speeds[A_HALF] = maxSpeed/2;
  A_Controller->speeds[A_QUARTER] = maxSpeed/4;

  //Set Distances
  //Set Starting Pos
  A_Controller->inflectionPoints[0] = *StartingPos;
  //Set Ending Pos
  A_Controller->inflectionPoints[7] = *EndingPos;

  //set distance to the distance between StartingPos and Ending Post
  MP_getDistanceAbs( StartingPos, EndingPos, &( A_Controller->distance) );
  
  //Set First Accel Point
  //The 8 represents the number of inflection points
  uint8_t firstInflectionMinor = (A_Controller->distance.posMinorStep)/(A_Controller->accelCoefficient*8);
  uint8_t firstInflectionMajor = (A_Controller->distance.posMajorStep)/(A_Controller->accelCoefficient*8);
  
  //This loop is fine as it is only done during init
  for(uint8_t i = 1; i < 7; i+=1)
  {
    //Sets the major and minor positions of  the inflection points to be equal to
    //start+ (i* dist(start,end)/(8*accelCoefficient) )
    A_Controller->inflectionPoints[i].posMinorStep = A_Controller->inflectionPoints[0].posMinorStep + (i*firstInflectionMinor);
    A_Controller->inflectionPoints[i].posMajorStep = A_Controller->inflectionPoints[0].posMajorStep + (i*firstInflectionMinor);
  }
   
}





#endif /* STEPPER_ACCELERATION_CONTROLLER_H */