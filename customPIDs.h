#ifndef CUSTOMPIDS_H
#define CUSTOMPIDS_H

#include "PIDOutput.h"
#include "Jaguar.h"

#define ENABLE_PID_ROTATION
#define ENABLE_PID_RANGE_FINDER
class GyroControlledTurning: public PIDOutput
{
private:
	Jaguar *jaguarFrontLeft;
	Jaguar *jaguarRearLeft;
	Jaguar *jaguarFrontRight;
	Jaguar *jaguarRearRight;
public:
	GyroControlledTurning(Jaguar *frontLeft, Jaguar *rearLeft, Jaguar *frontRight, Jaguar *rearRight);
	void PIDWrite(float output);
};

class SonarControlledDriving: public PIDOutput
{
private:
	Jaguar *jaguarFrontLeft;
	Jaguar *jaguarRearLeft;
	Jaguar *jaguarFrontRight;
	Jaguar *jaguarRearRight;
public:
	SonarControlledDriving(Jaguar *frontLeft, Jaguar *rearLeft, Jaguar *frontRight, Jaguar *rearRight);
	void PIDWrite(float output);
};

#endif
