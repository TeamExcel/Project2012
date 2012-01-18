#include "WPILib.h"
#include "IterativeRobot.h"

class Robot2012 : public IterativeRobot
{
private:
	RobotDrive myRobot; // robot drive system
	Joystick stickRightDrive; // only joystick
	Joystick stickLeftDrive;
	Joystick stickShooter;
public:
	Robot2012();
	
	void StartCompetition();
	
	void RobotInit();
	void DisabledInit();
	void AutonomousInit();
	void TeleopInit();
	
	void DisabledPeriodic();
	void AutonomousPeriodic();
	void TeleopPeriodic();
	
	void DisabledContinuous();
	void AutonomousContinuous();
	void TeleopContinuous();
};
