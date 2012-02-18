#include "AnalogRangeFinder.h"
#include "AnalogModule.h"
#include "AnalogChannel.h"

void AnalogRangeFinder::InitAnalogRangeFinder(void)
{
	inchesPerVolt = ANALOG_RANGE_FINDER_DEFAULT_INCHES_PER_VOLT;
}

AnalogRangeFinder::AnalogRangeFinder(UINT8 moduleNumber, UINT32 channel)
{
	m_analogChannel = new AnalogChannel(moduleNumber,channel);
	m_allocatedChannel = true;
	InitAnalogRangeFinder();
}

AnalogRangeFinder::AnalogRangeFinder(UINT32 channel)
{
	m_analogChannel = new AnalogChannel(channel);
	m_allocatedChannel = true;
	InitAnalogRangeFinder();
}

AnalogRangeFinder::AnalogRangeFinder(AnalogChannel *channel)
{
	if (channel != NULL)
	{
		m_analogChannel = channel;
		InitAnalogRangeFinder();
		m_allocatedChannel = false;
	}
}

AnalogRangeFinder::AnalogRangeFinder(AnalogChannel &channel)
{
	m_analogChannel = &channel;
	m_allocatedChannel = false;
	InitAnalogRangeFinder();
}

AnalogRangeFinder::~AnalogRangeFinder()
{
	if (m_allocatedChannel)
	{
		delete m_analogChannel;
	}
}

float AnalogRangeFinder::GetRangeInches()
{
	return m_analogChannel->GetVoltage() * inchesPerVolt;
}

float AnalogRangeFinder::GetVoltage()
{
	return m_analogChannel->GetVoltage();
}

void AnalogRangeFinder::SetSensitivity(float inches_per_volt)
{
	inchesPerVolt = inches_per_volt;
}


// PIDSource interface
double AnalogRangeFinder::PIDGet()
{
	return GetRangeInches();
}
	
