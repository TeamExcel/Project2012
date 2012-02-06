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

#define CAMERA_MAX_FPS 10
#define CAMERA_BRIGHTNESS_LEVEL 5
#define CAMERA_COLOR_LEVEL 100

#define SLOW_TURN 0.30F
#define FAST_TURN 0.40F

#define CENTER_OF_IMAGE 160

#define DEGREES_PER_PIXEL 0.16875F

#define AXIS_CAMERA_CONNECTED_TO_CRIO
//TODO setup IPs for the camera based on this #define


typedef enum
{
	PWM_CHANNEL_1_JAGUAR_FRONT_LEFT = 1,
	PWM_CHANNEL_2_JAGUAR_FRONT_RIGHT,
	PWM_CHANNEL_3_JAGUAR_REAR_LEFT,
	PWM_CHANNEL_4_JAGUAR_REAR_RIGHT,
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
	Gyro gyroscope;

	Timer timeInState;
	Timer timeSinceBoot;
	
	DriverStation *driverStation;
		
		// Local variables to count the number of periodic loops performed
	
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	bool cameraInitialized;
	float angleToTurn;
	float angleAtImage;
	bool anglesComputed;
	
		
public:

	Robot2012(void):
		jaguarFrontLeft(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_1_JAGUAR_FRONT_LEFT),
		jaguarFrontRight(DIGITAL_OUTPUT_CHANNEL,PWM_CHANNEL_2_JAGUAR_FRONT_RIGHT),
		myRobot(&jaguarFrontLeft,&jaguarFrontRight),
		stickRightDrive(1),
		stickLeftDrive(2),
		stickShooter(3),
		gyroscope(1),
		timeInState(),
		timeSinceBoot()
	{
		printf("Robot2012 Constructor Started\n");
		cameraInitialized = false;
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
		myRobot.SetExpiration(1.0);
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
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
	}

	void TeleopInit(void) 
	{
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		timeInState.Reset();
		timeInState.Start();
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);
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
		if (!stickRightDrive.GetTrigger())
		{
			anglesComputed = false;
			myRobot.TankDrive(stickLeftDrive,stickRightDrive);	
			//myRobot.SetSafetyEnabled(true);
			myRobot.SetSafetyEnabled(false);
		}
		CameraInitialize();
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");

		if (camera.IsFreshImage())
		{
			ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
			camera.GetImage(colorImage);
			if (stickRightDrive.GetTrigger())
			{
				FaceTarget(colorImage);
			}
			delete colorImage;
		}
	}


/********************************** Continuous Routines *************************************/
	void DisabledContinuous(void) 
	{
		//CameraInitialize();
	}

	void AutonomousContinuous(void)	
	{
		//AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		//CameraInitialize();
	}
	void TeleopContinuous(void) 
	{
		if ((stickRightDrive.GetTrigger()) && 
			(anglesComputed == true))
		{
			RotateToTarget();
		}
	}

	//returns 0 if a particle is found
	int FaceTarget(ColorImage *colorImage)
	{
		int returnVal = -1;
		BinaryImage *binaryImage;
		myRobot.SetSafetyEnabled(false);
		Image *imaqImage;
		
		if (stickRightDrive.GetTop())
		{
			colorImage->Write("capturedImage.jpg");
		}
		
		//binaryImage = colorImage->ThresholdHSV(0, 255, 0, 255, 255, 0);
		//binaryImage = colorImage->ThresholdHSV(56, 125, 55, 255, 255, 150);
		binaryImage = colorImage->ThresholdHSL(90, 115,30, 255, 70, 255);
		
		imaqImage = binaryImage->GetImaqImage();
		if (stickRightDrive.GetTop())
		{
			binaryImage->Write("afterCLRThreshold.bmp");
			IVA_ProcessImage(imaqImage, binaryImage);
			
		}	
		else
		{
			IVA_ProcessImage(imaqImage, (ImageBase *)0);
		}
		vector<ParticleAnalysisReport> *reports = binaryImage->GetOrderedParticleAnalysisReports();
		ParticleAnalysisReport *bottomParticlePtr = NULL;
		
		for (int particleIndex = 0; ((particleIndex <  reports->size()) && (particleIndex < 5)); particleIndex++)
		{
			ParticleAnalysisReport &thisReport = reports->at(particleIndex);
			if ((!bottomParticlePtr) || (thisReport.center_mass_y < bottomParticlePtr->center_mass_y))
			{
				bottomParticlePtr = &thisReport;
			}
		}
		if (bottomParticlePtr != NULL)
		{
			returnVal = 0; //indicates success
			int centerOfMassX = bottomParticlePtr->center_mass_x;
			angleToTurn = (centerOfMassX - CENTER_OF_IMAGE) * DEGREES_PER_PIXEL;
			angleAtImage = gyroscope.GetAngle();
			anglesComputed = true;
		}	
		else
		{	
			anglesComputed = false;
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
		}
		delete reports;
		delete binaryImage;

		return returnVal;
	}
	
	void RotateToTarget(void)
	{
		float angle_traveled = gyroscope.GetAngle() - angleAtImage;
		float angle_remaining = angleToTurn - angle_traveled;
		
		//turn left fast
		if (angle_remaining < -13.0)
		{
			jaguarFrontLeft.Set(-FAST_TURN);
			jaguarFrontRight.Set(-FAST_TURN);	
		}
		//turn left slowly
		else if (angle_remaining < -3.0)
		{
			jaguarFrontLeft.Set(-SLOW_TURN);
			jaguarFrontRight.Set(-SLOW_TURN);
		}
		//turn right fast
		else if (angle_remaining > 13.0)
		{
			jaguarFrontLeft.Set(FAST_TURN);
			jaguarFrontRight.Set(FAST_TURN);
		}
		//turn right slowly
		else if (angle_remaining > 3.0)
		{
			jaguarFrontLeft.Set(SLOW_TURN);
			jaguarFrontRight.Set(SLOW_TURN);
		}
		else
		{
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
		}
	}
	

	void CameraInitialize(void)
	{
		if (cameraInitialized == false)
		{
			AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
			if (&camera != (void *)0)
			{
				camera.WriteBrightness(CAMERA_BRIGHTNESS_LEVEL);
				camera.WriteColorLevel(CAMERA_COLOR_LEVEL);
				camera.WriteMaxFPS(CAMERA_MAX_FPS);
			}
		}
	}
};
START_ROBOT_CLASS(Robot2012);
