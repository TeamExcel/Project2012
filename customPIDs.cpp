#include "customPIDs.h"


GyroControlledTurning::GyroControlledTurning(Jaguar *frontLeft, Jaguar *rearLeft, Jaguar *frontRight, Jaguar *rearRight)
{
	jaguarFrontLeft = frontLeft;
	jaguarRearLeft = rearLeft;
	jaguarFrontRight = frontRight;
	jaguarRearRight = rearRight;
}

#include "DriverStationLCD.h"
void GyroControlledTurning::PIDWrite(float output)
{
	DriverStationLCD *driverStationLCD = DriverStationLCD::GetInstance();
	driverStationLCD->PrintfLine((DriverStationLCD::Line) 2, "PID Output: %f", output);
	output = -output;
	#ifdef ENABLE_PID_ROTATION
	if (jaguarFrontLeft)
		jaguarFrontLeft->Set(output);
	if (jaguarRearLeft)
		jaguarRearLeft->Set(output);
	if (jaguarFrontRight)
		jaguarFrontRight->Set(output);
	if (jaguarRearRight)
		jaguarRearRight->Set(output);
	#endif
}


SonarControlledDriving::SonarControlledDriving(Jaguar *frontLeft, Jaguar *rearLeft, Jaguar *frontRight, Jaguar *rearRight)
{
	jaguarFrontLeft = frontLeft;
	jaguarRearLeft = rearLeft;
	jaguarFrontRight = frontRight;
	jaguarRearRight = rearRight;
}

void SonarControlledDriving::PIDWrite(float output)
{
	#ifdef ENABLE_PID_ROTATION
	if (jaguarFrontLeft)
		jaguarFrontLeft->Set(output);
	if (jaguarRearLeft)
		jaguarRearLeft->Set(output);
	if (jaguarFrontRight)
		jaguarFrontRight->Set(-output);
	if (jaguarRearRight)
		jaguarRearRight->Set(-output);
	#endif
}
