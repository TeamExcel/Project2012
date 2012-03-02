#include "customPIDs.h"
#include "DriverStationLCD.h"


GyroControlledTurning::GyroControlledTurning(RobotDrive *controlledRobot)
{
	theRobot = controlledRobot;
}

void GyroControlledTurning::PIDWrite(float output)
{
	if (theRobot)
		theRobot->TankDrive(-output, -output);
}


SonarControlledDriving::SonarControlledDriving(RobotDrive *controlledRobot)
{
	theRobot = controlledRobot;
}

void SonarControlledDriving::PIDWrite(float output)
{
	DriverStationLCD *driverStationLCD = DriverStationLCD::GetInstance();
	driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "PID Output: %f", output);
	if (theRobot)
		theRobot->TankDrive(output, -output);
}
