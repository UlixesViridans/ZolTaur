#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H

typedef struct
{
  //Int that is 0-199 that represents major steps
  uint8_t posMajorStep;
  //int that is is 0-255 that represents microsteps that make up a major step
  uint8_t posMinorStep;
}MotorPosition_TMC2208;

//Takes in three MotorPosition Pointers:
//Distance 1
//Distance 2
//Result
// Result == Distance1 - Distance2
//The position pointed to by result gets modified as
//A result of this operation
void MP_getDistanceAbs(MotorPosition_TMC2208 * Dist1, MotorPosition_TMC2208 * Dist2, MotorPosition_TMC2208 * Result )
{
  //Minor
  uint8_t res1 = Dist1->posMinorStep - Dist2->posMinorStep;
  
  //Major
  uint8_t res2;

  //Detect Underflow
  //If the result is bigger than 
  //Dist1 then an underflow happened
  if(res1 > Dist1->posMinorStep)
  {
    res2 = Dist1->posMajorStep - ( Dist2->posMajorStep + 1 );
  }
  else
  {
    res2 = Dist1->posMajorStep - Dist2->posMajorStep;
  }

  //Detect Underflow in major step
  if(res2 > Dist1->posMajorStep )
  {
    //if there's an underflow subtract 56 from
    //res2 to renormalize it to the 0-199 scale of
    //posMajorStep
    res2-=56;
  }

  //set result
  Result->posMinorStep = res1;
  Result->posMajorStep = res2;
}

//Not ready, dont use
void MP_add_Positions( MotorPosition_TMC2208 * Dist1, MotorPosition_TMC2208 * Dist2, MotorPosition_TMC2208 * Result )
{
  uint8_t resMinor = Dist1->posMinorStep + Dist2->posMinorStep;
  uint8_t resMajor;
  //Detect overflow
  if(resMinor < Dist1->posMinorStep)
  {
    resMajor = Dist1->posMajorStep + Dist2->posMajorStep + 1;
  }
  else
  {
    resMajor = Dist1->posMajorStep + Dist2->posMajorStep;
  }

  //If major step is over 199 then overflow
  if(resMajor >= 200)
  {
    resMajor -= 200;
  }

  
}

//Compare Two Motor Positions, 
// if ptr1 > ptr2 return 1
// if ptr1 == ptr2 return 0
// if ptr1 < ptr2 return -1
uint8_t compareMotorPosition_TMC2208( MotorPosition_TMC2208 * pos1, MotorPosition_TMC2208 * pos2 )
{
  if( pos1->posMajorStep == pos2->posMajorStep )
  {
    if( pos1->posMinorStep == pos2->posMinorStep )
    {
      return 0;
    }
    else if( pos1->posMinorStep > pos2->posMinorStep )
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
  else if( pos1->posMajorStep > pos2->posMajorStep )
  {
    return 1;
  }
  else
  {
    return -1;
  }
}

#endif /*MOTOR_POSITION_H*/