#include "Robot2012.h"
#include "Vision/AxisCamera.h"

////////////////////////////////////////////////////////
// Defines and typedefs
////////////////////////////////////////////////////////
#define ANALOG_OUTPUT_CHANNEL 1		//in 2012 this should be in slot 1 (chan 1), or slot 5 (chan 2)
#define DIGITAL_OUTPUT_CHANNEL 2	//in 2012 this should be in slot 3 (chan 1), or slot 7 (chan 2)
#define PNEUMATIC_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 2 (chan 1), or slot 6 (chan 2)

typedef enum
{
	PWM_CHANNEL_1_JAGUAR_FRONT_LEFT = 1,
	PWM_CHANNEL_2_JAGUAR_FRONT_RIGHT,
	PWM_CHANNEL_3_UNUSED,
	PWM_CHANNEL_4_UNUSED,
	PWM_CHANNEL_5_UNUSED,
	PWM_CHANNEL_6_UNUSED,
	PWM_CHANNEL_7_UNUSED,
	PWM_CHANNEL_8_UNUSED,
	PWM_CHANNEL_9_UNUSED,
}PWM_CHANNEL_TYPE;

////////////////////////////////////////////////////////
//  Class Methods
////////////////////////////////////////////////////////
Robot2012::Robot2012(void):
		jaguarFrontLeft(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_1_JAGUAR_FRONT_LEFT),
		jaguarFrontRight(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_2_JAGUAR_FRONT_RIGHT),
		myRobot(&jaguarFrontLeft, &jaguarFrontRight),	// these must be initialized in the same order
		stickRightDrive(1),		// as they are declared above
		stickLeftDrive(2),
		stickShooter(3)
{
	myRobot.SetSafetyEnabled(false);
//	myRobot.SetExpiration(0.1);
	//camera = *(AxisCamera::GetInstance("10.24.74.11"));
}

void Robot2012::StartCompetition()
{
	
}

void Robot2012::RobotInit()
{

	myRobot.SetSafetyEnabled(false);	
}

void Robot2012::DisabledInit()
{

	myRobot.SetSafetyEnabled(false);
}

void Robot2012::AutonomousInit()
{

	myRobot.SetSafetyEnabled(false);
}

void Robot2012::TeleopInit()
{

	myRobot.SetSafetyEnabled(false);
//	myRobot.SetExpiration(0.1);
//	myRobot.SetSafetyEnabled(true);
}

void Robot2012::DisabledPeriodic()
{
	
}

void Robot2012::AutonomousPeriodic()
{
	//AxisCamera &camera = AxisCamera::GetInstance();
	myRobot.SetSafetyEnabled(false);
//	myRobot.Drive(0.5, 0.0); 	// drive forwards half speed
//	Wait(2.0); 				//    for 2 seconds
//	myRobot.Drive(0.0, 0.0); 	// stop robot
}

void Robot2012::TeleopPeriodic()
{
	//AxisCamera &camera = AxisCamera::GetInstance();
}

void Robot2012::DisabledContinuous()
{
	
}

void Robot2012::AutonomousContinuous()
{
	
}

void Robot2012::TeleopContinuous()
{
	myRobot.TankDrive(stickLeftDrive, stickRightDrive); // drive with arcade style (use right stick)

}
///////////////////////////////////////////////////////
// Non-class functions
///////////////////////////////////////////////////////
START_ROBOT_CLASS(Robot2012);

