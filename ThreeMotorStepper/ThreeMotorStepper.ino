#include <AccelStepper.h>

#define EN        8


AccelStepper stepper1(AccelStepper::DRIVER, 7, 4);
AccelStepper stepper2(AccelStepper::DRIVER, 6, 3);
AccelStepper stepper3(AccelStepper::DRIVER, 5, 2);
void setup()
{  
    pinMode(EN, OUTPUT);
    digitalWrite(EN, LOW);
    stepper1.setMaxSpeed(3000);

    stepper1.setAcceleration(3000);
    stepper1.moveTo(10000);
    
    stepper2.setMaxSpeed(350);

    stepper2.setAcceleration(500);
    stepper2.moveTo(10000);
    
    stepper3.setMaxSpeed(350);

    stepper3.setAcceleration(500);
    stepper3.moveTo(10000); 
}
void loop()
{
    // Change direction at the limits
    if (stepper1.distanceToGo() == 0)
        stepper1.moveTo(-stepper1.currentPosition());
    if (stepper2.distanceToGo() == 0)
        stepper2.moveTo(-stepper2.currentPosition());
    if (stepper3.distanceToGo() == 0)
        stepper3.moveTo(-stepper3.currentPosition());
    stepper1.run();
    //stepper2.run();
    //stepper3.run();
}
