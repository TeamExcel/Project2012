#include "WPILib.h"
#include "SimpleCRobot.h"
#include "CRobotDrive.h"
#include "Robot2012.h"

#define FRONT_LEFT_MOTOR_PORT 0
#define FRONT_RIGHT_MOTOR_PORT 0
#define REAR_LEFT_MOTOR_PORT 0
#define REAR_RIGHT_MOTOR_PORT 0

#define LOWER_ROLLER_PORT 0
#define UPPER_ROLLER_PORT 0
#define CATAPULT_THROW 0
#define CATAPULT_WITHDRAW 0

static const UINT32 LEFT_MOTOR_PORT = 1;
static const UINT32 RIGHT_MOTOR_PORT = 2;
static const UINT32 JOYSTICK_PORT = 1;

void Initialize(void)
{
	CreateRobotDrive(FRONT_LEFT_MOTOR_PORT, REAR_LEFT_MOTOR_PORT, 
			FRONT_RIGHT_MOTOR_PORT, REAR_RIGHT_MOTOR_PORT);
	SetWatchdogExpiration(0.1);
}

void Autonomous(void)
{
	SetWatchdogEnabled(false);
	Drive(0.5, 0.0);
	Wait(2.0);
	Drive(0.0, 0.0);
}

void OperatorControl(void)
{
	SetWatchdogEnabled(true);
	while (IsOperatorControl())
	{
		WatchdogFeed();
		ArcadeDrive(JOYSTICK_PORT);
		Wait(0.005);
	}
}

START_ROBOT_CLASS(SimpleCRobot);
