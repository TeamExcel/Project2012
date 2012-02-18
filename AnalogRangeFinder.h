#ifndef ANALOGRANGEFINDER_H
#define ANALOGRANGEFINDER_H

#include "SensorBase.h"
#include "PIDSource.h"

class AnalogChannel;
class AnalogModule;

#define ANALOG_RANGE_FINDER_DEFAULT_INCHES_PER_MV  0.1024F
#define ANALOG_RANGE_FINDER_DEFAULT_INCHES_PER_VOLT  (0.1024F * 1000.0F)


//Designed for the MaxBotix LV-MaxSonar®-EZ1, but should work for any analog output sonar module
class AnalogRangeFinder : public SensorBase, public PIDSource
{
private:
	float inchesPerVolt;
	AnalogChannel *m_analogChannel;
	bool m_allocatedChannel;
	
	void InitAnalogRangeFinder(void);
	
public:
	AnalogRangeFinder(UINT8 moduleNumber, UINT32 channel);
	explicit AnalogRangeFinder(UINT32 channel);
	explicit AnalogRangeFinder(AnalogChannel *channel);
	explicit AnalogRangeFinder(AnalogChannel &channel);
	virtual ~AnalogRangeFinder();
	
	virtual float GetRangeInches();
	float GetVoltage();
	void SetSensitivity(float inches_per_volt);
	
	// PIDSource interface
	double PIDGet();
	
};


	
#endif
