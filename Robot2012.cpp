#include "WPILib.h"
//#include "Robot2012.h"
#include "Vision/AxisCamera.h"
#include "nivision.h"
#include "ImageProcessing.h"
////////////////////////////////////////////////////////
// Defines and typedefs
////////////////////////////////////////////////////////
#define ANALOG_OUTPUT_CHANNEL 1		//in 2012 this should be in slot 1 (chan 1), or slot 5 (chan 2)
#define DIGITAL_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 3 (chan 1), or slot 7 (chan 2)
#define PNEUMATIC_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 2 (chan 1), or slot 6 (chan 2)

#define CAMERA_MAX_FPS 5

#define AXIS_CAMERA_CONNECTED_TO_CRIO
//TODO setup IPs for the camera based on this #define


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

class Robot2012 : public IterativeRobot
{
	Jaguar jaguarFrontLeft;
	Jaguar jaguarFrontRight;
	RobotDrive myRobot; // robot drive system
	Joystick stickRightDrive; // only joystick
	Joystick stickLeftDrive;
	Joystick stickShooter;

	Timer timeInState;
	Timer timeSinceBoot;
	
	DriverStation *driverStation;
		
		// Local variables to count the number of periodic loops performed
	
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
		
public:

	Robot2012(void):
		jaguarFrontLeft(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_1_JAGUAR_FRONT_LEFT),
		jaguarFrontRight(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_2_JAGUAR_FRONT_RIGHT),
		myRobot(&jaguarFrontLeft,&jaguarFrontRight),
		stickRightDrive(1),
		stickLeftDrive(2),
		stickShooter(3),
		timeInState(),
		timeSinceBoot()
	{
		printf("Robot2012 Constructor Started\n");

		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();

		// Iterate over all the buttons on each joystick, setting state to false for each
//		UINT8 buttonNum = 1;						// start counting buttons at button 1
//		for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
//			m_rightStickButtonState[buttonNum] = false;
//			m_leftStickButtonState[buttonNum] = false;
//		}

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;

		printf("Robot2012 Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) 
	{
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
		
		timeSinceBoot.Start();
	}
	
	void DisabledInit(void) 
	{
		timeInState.Reset();
		timeInState.Start();
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
	}

	void AutonomousInit(void) 
	{
		timeInState.Reset();
		timeInState.Start();
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
	}

	void TeleopInit(void) 
	{
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		timeInState.Reset();
		timeInState.Start();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  
	{
		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
	}

	void AutonomousPeriodic(void) 
	{
		m_autoPeriodicLoops++;

				
		/* the below code (if uncommented) would drive the robot forward at half speed
		 * for two seconds.  This code is provided as an example of how to drive the 
		 * robot in autonomous mode, but is not enabled in the default code in order
		 * to prevent an unsuspecting team from having their robot drive autonomously!
		 */
		/* below code commented out for safety
		if (m_autoPeriodicLoops == 1) {
			// When on the first periodic loop in autonomous mode, start driving forwards at half speed
			m_robotDrive->Drive(0.5, 0.0);			// drive forwards at half speed
		}
		if (m_autoPeriodicLoops == (2 * GetLoopsPerSec())) {
			// After 2 seconds, stop the robot 
			m_robotDrive->Drive(0.0, 0.0);			// stop robot
		}
		*/
	}

	
	void TeleopPeriodic(void) 
	{
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		myRobot.TankDrive(stickLeftDrive,stickRightDrive);	
	}


/********************************** Continuous Routines *************************************/
	void DisabledContinuous(void) 
	{
	}

	void AutonomousContinuous(void)	
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		if (camera.GetMaxFPS() != CAMERA_MAX_FPS)
		{
			camera.WriteMaxFPS(CAMERA_MAX_FPS);
		}
	}
	void TeleopContinuous(void) 
	{
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		if (camera.GetMaxFPS() != CAMERA_MAX_FPS)
		{
			camera.WriteMaxFPS(CAMERA_MAX_FPS);
		}
		if (camera.IsFreshImage())
		{
			ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_RGB);
			camera.GetImage(colorImage);
			Image *image = colorImage->GetImaqImage();
			IVA_Data *ivaData = IVA_InitData(4, 0);
			
			IVA_ProcessImage(image, ivaData);
			
			//TODO retrieve useful data from ivaData
			
			IVA_DisposeData(ivaData);
			
		}
	}

				
};

START_ROBOT_CLASS(Robot2012);
