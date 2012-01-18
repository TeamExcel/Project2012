#include "Robot2012.h"
#include "Vision/AxisCamera.h"

//AxisCamera *camera;

Robot2012::Robot2012(void):
		myRobot(1, 2),	// these must be initialized in the same order
		stickRightDrive(1),		// as they are declared above
		stickLeftDrive(2),
		stickShooter(3)
{
	myRobot.SetExpiration(0.1);
	//camera = *(AxisCamera::GetInstance("10.24.74.11"));
}

void Robot2012::StartCompetition()
{
	
}

void Robot2012::RobotInit()
{
	
}

void Robot2012::DisabledInit()
{
	
}

void Robot2012::AutonomousInit()
{
	
}

void Robot2012::TeleopInit()
{
	
}

void Robot2012::DisabledPeriodic()
{
	
}

void Robot2012::AutonomousPeriodic()
{
	AxisCamera &camera = AxisCamera::GetInstance();
	myRobot.SetSafetyEnabled(false);
//	myRobot.Drive(0.5, 0.0); 	// drive forwards half speed
//	Wait(2.0); 				//    for 2 seconds
//	myRobot.Drive(0.0, 0.0); 	// stop robot
}

void Robot2012::TeleopPeriodic()
{
	AxisCamera &camera = AxisCamera::GetInstance();
	myRobot.SetSafetyEnabled(true);
	myRobot.TankDrive(stickLeftDrive, stickRightDrive); // drive with arcade style (use right stick)
	
}

void Robot2012::DisabledContinuous()
{
	
}

void Robot2012::AutonomousContinuous()
{
	
}

void Robot2012::TeleopContinuous()
{
	
}

START_ROBOT_CLASS(Robot2012);

