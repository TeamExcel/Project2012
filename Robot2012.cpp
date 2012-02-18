#include "WPILib.h"
//#include "Robot2012.h"
#include "Vision/AxisCamera.h"
#include "nivision.h"
#include "ImageProcessing.h"
#include "customPIDs.h"
#include "AnalogRangeFinder.h"
////////////////////////////////////////////////////////
// Defines and typedefs
////////////////////////////////////////////////////////
#define ANALOG_OUTPUT_CHANNEL 1		//in 2012 this should be in slot 1 (chan 1), or slot 5 (chan 2)
#define DIGITAL_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 2 (chan 1), or slot 7 (chan 2)
#define SOLENOID_OUTPUT_CHANNEL 1	//in 2012 this should be in slot 3 (chan 1), or slot 6 (chan 2)

#define CAMERA_MAX_FPS 10
#define CAMERA_BRIGHTNESS_LEVEL 5
#define CAMERA_COLOR_LEVEL 100

#define VERY_SLOW_TURN 0.20F
#define SLOW_TURN 0.30F
#define FAST_TURN 0.40F

#define ELEVATOR_SPEED_BOTTOM 0.6F
#define ELEVATOR_SPEED_TOP 1.0F




#define CENTER_OF_IMAGE 160



#define HALF_VERTICAL_FIELD_OF_VIEW 19.13711F
#define TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW .347007F

#define HALF_HORIZONTAL_FIELD_OF_VIEW 24.40828F
#define TARGET_HEIGHT_IN_FEET 1.5F
#define IMAGE_HEIGHT_IN_PIXELS 240


#define IMAGE_HEIGHT_IN_FEET(target_height_pxls) ((TARGET_HEIGHT_IN_FEET * (IMAGE_HEIGHT_IN_PIXELS / 2)) / target_height_pxls)
#define CALCULATE_DISTANCE(target_height_pxls) (IMAGE_HEIGHT_IN_FEET(target_height_pxls)/TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW)
#define HORIZONTAL_DEGREES_PER_PIXEL (HALF_HORIZONTAL_FIELD_OF_VIEW/CENTER_OF_IMAGE)


//Controls defines - for new buttons, add a #define here and use it to get the key you want, that way we can change controls easily
#define BUTTON_CAMERA_ALIGN_SHOT_BUTTON() stickRightDrive.GetTrigger()
#define BUTTON_CAMERA_TAKE_DEBUG_PICTURES() 0

#define BUTTON_ELEVATOR_BOTTOM_UP() (stickShooter.GetRawButton(2) || stickRightDrive.GetTop())
#define BUTTON_ELEVATOR_BOTTOM_DOWN() stickShooter.GetRawButton(3)
#define BUTTON_ELEVATOR_TOP_UP() stickShooter.GetRawButton(4)
#define BUTTON_ELEVATOR_TOP_DOWN() stickShooter.GetRawButton(5)

#define BUTTON_CATAPULT_SHOOT() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_LATCH() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_FORCE_SHOOT() stickShooter.GetRawButton(10)
#define THROTTLE_ELEVATORS() (((-stickShooter.GetThrottle()) + 1) / 2)

#define BUTTON_DUMPER_RAMP_EXTEND() stickShooter.GetRawButton(9)
#define BUTTON_DUMPER_ROLLER() stickShooter.GetRawButton(11)

#define BUTTON_LOWER_BRIDGE_RAM() stickLeftDrive.GetTop()


//PID Parameters
#define ROTATION_PID_PROPORTION 2.0
#define ROTATION_PID_INTEGRAL 1.0
#define ROTATION_PID_DERIVATIVE 1.0

#define ROTATION_PID_MIN_INPUT -30.0
#define ROTATION_PID_MAX_INPUT 30.0
#define ROTATION_PID_MIN_OUTPUT -0.5
#define ROTATION_PID_MAX_OUTPUT 0.5

#define ROTATION_PID_TOLERENCE 4.0 //4 percent

#define RANGE_PID_PROPORTION 2.0
#define RANGE_PID_INTEGRAL 1.0
#define RANGE_PID_DERIVATIVE 1.0

#define RANGE_PID_MIN_INPUT 0.0
#define RANGE_PID_MAX_INPUT 240.0
#define RANGE_PID_MIN_OUTPUT -0.5
#define RANGE_PID_MAX_OUTPUT 0.5

#define RANGE_PID_SETPOINT 156.0
#define RANGE_PID_TOLERENCE 5.0  

//Uncomment this to enable the PID tuning section of code that can help tune PIDs
//by running code in debug mode and using breakpoints.
//#define PID_TUNING

//Helper Macros
#define DUMPER_RAMP_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidDumperRampUp, &solenoidDumperRampDown);}
#define BRIDGE_RAM_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidBridgeRamDown, &solenoidBridgeRamUp);}
#define CATAPULT_PUSHER_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultPusher, &solenoidCatapultPuller);}
#define CATAPULT_LATCH_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultLatchExtend, &solenoidCatapultLatchRetract);}

typedef enum
{
	PWM_CHANNEL_1_JAGUAR_REAR_RIGHT = 1,
	PWM_CHANNEL_2_JAGUAR_REAR_LEFT,
	PWM_CHANNEL_3_JAGUAR_FRONT_RIGHT,
	PWM_CHANNEL_4_JAGUAR_FRONT_LEFT,
	PWM_CHANNEL_5_JAGUAR_ELEVATOR_BOTTOM_1,
	PWM_CHANNEL_6_JAGUAR_ELEVATOR_BOTTOM_2,
	PWM_CHANNEL_7_JAGUAR_ELEVATOR_TOP,
	PWM_CHANNEL_8_JAGUAR_DUMPER_ROLLER,
	PWM_CHANNEL_9_UNUSED
}PWM_CHANNEL_TYPE;

#define BOTTOM_ROLLERS_SYNC_GROUP 1

typedef enum
{
	SOLENOID_CHANNEL_1_CATAPULT_PUSHER = 1,
	SOLENOID_CHANNEL_2_CATAPULT_PULLER,
	SOLENOID_CHANNEL_3_BRIDGE_RAM_DOWN,
	SOLENOID_CHANNEL_4_BRIDGE_RAM_UP,
	SOLENOID_CHANNEL_5_DUMPER_RAMP_UP,
	SOLENOID_CHANNEL_6_DUMPER_RAMP_DOWN,
	SOLENOID_CHANNEL_7_CATAPULT_LATCH_EXTEND,
	SOLENOID_CHANNEL_8_CATAPULT_LATCH_RETRACT
}SOLENOID_CHANNEL_TYPE;

typedef enum
{
	ANALOG_CHANNEL_1_GYROSCOPE = 1,
	ANALOG_CHANNEL_2_RANGE_FINDER,
	ANALOG_CHANNEL_3_UNUSED,
	ANALOG_CHANNEL_4_UNUSED,
	ANALOG_CHANNEL_5_UNUSED,
	ANALOG_CHANNEL_6_UNUSED,
	ANALOG_CHANNEL_7_UNUSED,
	ANALOG_CHANNEL_8_UNUSED,
	ANALOG_CHANNEL_9_UNUSED
}ANALOG_CHANNEL_TYPE;

typedef enum
{
	DIGITAL_CHANNEL_1_INPUT_COMPRESSOR_SWITCH = 1,
	DIGITAL_CHANNEL_2_UNUSED,
	DIGITAL_CHANNEL_3_UNUSED,
	DIGITAL_CHANNEL_4_UNUSED,
	DIGITAL_CHANNEL_5_UNUSED,
	DIGITAL_CHANNEL_6_UNUSED,
	DIGITAL_CHANNEL_7_UNUSED,
	DIGITAL_CHANNEL_8_UNUSED
}DIGITAL_IO_CHANNEL_TYPE;

typedef enum 
{
	RELAY_CHANNEL_1_COMPRESSOR_RELAY = 1,
	RELAY_CHANNEL_2_UNUSED,
	RELAY_CHANNEL_3_UNUSED,
	RELAY_CHANNEL_4_UNUSED,
	RELAY_CHANNEL_5_UNUSED,
	RELAY_CHANNEL_6_UNUSED,
	RELAY_CHANNEL_7_UNUSED,
	RELAY_CHANNEL_8_UNUSED,
}RELAY_CHANNEL_TYPE;

class Robot2012 : public IterativeRobot
{
	Jaguar jaguarFrontLeft;
	Jaguar jaguarFrontRight;
	Jaguar jaguarRearLeft;
	Jaguar jaguarRearRight;
	RobotDrive myRobot; // robot drive system
	Jaguar jaguarElevatorBottom1;
	Jaguar jaguarElevatorBottom2;
	Jaguar jaguarElevatorTop;
	Jaguar jaguarDumperRoller;
	Solenoid solenoidCatapultPusher;
	Solenoid solenoidCatapultPuller;
	Solenoid solenoidBridgeRamDown;
	Solenoid solenoidBridgeRamUp;
	Solenoid solenoidDumperRampUp;
	Solenoid solenoidDumperRampDown;
	Solenoid solenoidCatapultLatchExtend;
	Solenoid solenoidCatapultLatchRetract;
	Joystick stickRightDrive; // only joystick
	Joystick stickLeftDrive;
	Joystick stickShooter;
	Gyro gyroscope;
#ifdef ENABLE_PID_ROTATION
	GyroControlledTurning rotationControl;
	PIDController rotationPID;
#endif
#ifdef ENABLE_PID_RANGE_FINDER
	AnalogRangeFinder rangeFinder;
	SonarControlledDriving sonarDriveControl;
	PIDController rangePID;
#endif
	Compressor compressor;

	Timer timeInState;
	Timer timeSinceBoot;
	
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
		
		// Local variables to count the number of periodic loops performed
	
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	bool cameraInitialized;
	float angleToTurn;
	float angleAtImage;
	bool anglesComputed;
	int testCount;
	float distanceToTarget;
		
public:

	Robot2012(void):
		jaguarFrontLeft(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_4_JAGUAR_FRONT_LEFT),
		jaguarFrontRight(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_3_JAGUAR_FRONT_RIGHT),
		jaguarRearLeft(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_2_JAGUAR_REAR_LEFT),
		jaguarRearRight(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_1_JAGUAR_REAR_RIGHT),
		myRobot(&jaguarFrontLeft, &jaguarRearLeft, &jaguarFrontRight, &jaguarRearRight),
		jaguarElevatorBottom1(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_5_JAGUAR_ELEVATOR_BOTTOM_1),
		jaguarElevatorBottom2(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_6_JAGUAR_ELEVATOR_BOTTOM_2),
		jaguarElevatorTop(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_7_JAGUAR_ELEVATOR_TOP),
		jaguarDumperRoller(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_8_JAGUAR_DUMPER_ROLLER),
		solenoidCatapultPusher(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_1_CATAPULT_PUSHER),
		solenoidCatapultPuller(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_2_CATAPULT_PULLER),
		solenoidBridgeRamDown(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_3_BRIDGE_RAM_DOWN),
		solenoidBridgeRamUp(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_4_BRIDGE_RAM_UP),
		solenoidDumperRampUp(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_5_DUMPER_RAMP_UP),
		solenoidDumperRampDown(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_6_DUMPER_RAMP_DOWN),
		solenoidCatapultLatchExtend(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_7_CATAPULT_LATCH_EXTEND),
		solenoidCatapultLatchRetract(SOLENOID_OUTPUT_CHANNEL, SOLENOID_CHANNEL_8_CATAPULT_LATCH_RETRACT),
		stickRightDrive(1),
		stickLeftDrive(2),
		stickShooter(3),
		gyroscope(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_1_GYROSCOPE),

#ifdef ENABLE_PID_ROTATION
		rotationControl(&jaguarFrontLeft, &jaguarRearLeft, &jaguarFrontRight, &jaguarRearRight),
		rotationPID(ROTATION_PID_PROPORTION, ROTATION_PID_INTEGRAL, ROTATION_PID_DERIVATIVE, &gyroscope, &rotationControl),
#endif
#ifdef ENABLE_PID_RANGE_FINDER
		rangeFinder(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_2_RANGE_FINDER),
		sonarDriveControl(&jaguarFrontLeft, &jaguarRearLeft, &jaguarFrontRight, &jaguarRearRight),
		rangePID(RANGE_PID_PROPORTION,RANGE_PID_INTEGRAL,RANGE_PID_DERIVATIVE, &rangeFinder, &sonarDriveControl),
#endif		
		compressor(DIGITAL_CHANNEL_1_INPUT_COMPRESSOR_SWITCH, RELAY_CHANNEL_1_COMPRESSOR_RELAY),
		timeInState(),
		timeSinceBoot()
	{
		printf("Robot2012 Constructor Started\n");
		cameraInitialized = false;
		// Acquire the Driver Station object
		driverStation = DriverStation::GetInstance();
		driverStationLCD = DriverStationLCD::GetInstance();
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kFrontRightMotor, true);

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
#ifdef ENABLE_PID_ROTATION
		rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
		rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
		rotationPID.SetTolerance(ROTATION_PID_TOLERENCE);
		rotationPID.Disable();
#endif
#ifdef ENABLE_PID_RANGE_FINDER
		rangePID.SetInputRange(RANGE_PID_MIN_INPUT,RANGE_PID_MAX_INPUT);
		rangePID.SetOutputRange(RANGE_PID_MIN_OUTPUT, RANGE_PID_MAX_OUTPUT);
		rangePID.SetSetpoint(RANGE_PID_SETPOINT);
		rangePID.SetTolerance(RANGE_PID_TOLERENCE);
		rangePID.Disable();
#endif
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
		compressor.Stop();
	}

	void AutonomousInit(void) 
	{
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		anglesComputed = false;
	}

	void TeleopInit(void) 
	{
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		anglesComputed = false;
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  
	{
		m_disabledPeriodicLoops++;
	}

	void AutonomousPeriodic(void) 
	{
		m_autoPeriodicLoops++;

	}

	
	void TeleopPeriodic(void) 
	{
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		
		if (!BUTTON_CAMERA_ALIGN_SHOT_BUTTON())
		{
			anglesComputed = false;
#ifdef ENABLE_PID_ROTATION
			rotationPID.Disable();
#endif
			myRobot.TankDrive(stickLeftDrive,stickRightDrive);	
			//myRobot.SetSafetyEnabled(true);
			myRobot.SetSafetyEnabled(false);
			testCount = 0;
		}
		CameraInitialize();

		
		ManageAppendages();
		ManageElevator();
		ManageCatapult();

	#ifndef ENABLE_PID_ROTATION
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		if (camera.IsFreshImage())
		{
			ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
			camera.GetImage(colorImage);
			if (BUTTON_CAMERA_ALIGN_SHOT_BUTTON() && (anglesComputed == false))
			{
				gyroscope.Reset();
				DetermineTargetPosition(colorImage);
			}
			delete colorImage;
		}
		#endif
		PositionForTarget();
		driverStationLCD->UpdateLCD();
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
	#ifndef ENABLE_PID_ROTATION
		if ((BUTTON_CAMERA_ALIGN_SHOT_BUTTON()) && 
			(anglesComputed == true))
		{
			RotateToTarget();
		}
	#endif
	}

	//returns 0 if a particle is found
	int DetermineTargetPosition(ColorImage *colorImage)
	{
		int returnVal = -1;
		BinaryImage *binaryImage;
		myRobot.SetSafetyEnabled(false);
		Image *imaqImage;
		
		if (BUTTON_CAMERA_TAKE_DEBUG_PICTURES())
		{
			colorImage->Write("capturedImage.jpg");
		}
		
		//binaryImage = colorImage->ThresholdHSV(0, 255, 0, 255, 255, 0);
		//binaryImage = colorImage->ThresholdHSV(56, 125, 55, 255, 255, 150);
		binaryImage = colorImage->ThresholdHSL(90, 115,30, 255, 70, 255);
		
		imaqImage = binaryImage->GetImaqImage();
		if (BUTTON_CAMERA_TAKE_DEBUG_PICTURES())
		{
			binaryImage->Write("afterCLRThreshold.bmp");
			IVA_ProcessImage(imaqImage, binaryImage);
		}	
		else
		{
			IVA_ProcessImage(imaqImage, (ImageBase *)0);
		}
		vector<ParticleAnalysisReport> *reports = binaryImage->GetOrderedParticleAnalysisReports();
		ParticleAnalysisReport *topParticlePtr = NULL;
		
		for (int particleIndex = 0; ((particleIndex <  reports->size()) && (particleIndex < 5)); particleIndex++)
		{
			ParticleAnalysisReport &thisReport = reports->at(particleIndex);
			if ((!topParticlePtr) || (thisReport.center_mass_y  < topParticlePtr->center_mass_y))
			{
				topParticlePtr = &thisReport;
			}
		}
		if (topParticlePtr != NULL)
		{
			returnVal = 0; //indicates success
			int centerOfMassX = topParticlePtr->center_mass_x;
			
			angleToTurn = (centerOfMassX - CENTER_OF_IMAGE) * HORIZONTAL_DEGREES_PER_PIXEL;
			
			distanceToTarget = CALCULATE_DISTANCE(topParticlePtr->boundingRect.height);

			driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Angle to turn: %.5f", angleToTurn);
			//driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Distance: %.5f", distanceToTarget);
			driverStationLCD->UpdateLCD();
			anglesComputed = true;

#ifdef ENABLE_PID_ROTATION		
			rotationPID.SetSetpoint(-angleToTurn);
			#ifdef PID_TUNING
				float p = rotationPID.GetP();
				float i = rotationPID.GetI();
				float d = rotationPID.GetD();
				rotationPID.SetPID(p,i,d);
				
				p = rangePID.GetP();
				i = rangePID.GetI();
				d = rangePID.GetD();
				rangePID.SetPID(p,i,d);
			#endif
			
			
			
			rotationPID.Enable();
#endif
		}	
		else
		{	
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
			jaguarRearLeft.Set(0.0);
			jaguarRearRight.Set(0.0);
			anglesComputed = false;
		}
		delete reports;
		delete binaryImage;

		return returnVal;
	}
	
	void RotateToTarget(void)
	{
#ifndef ENABLE_PID_ROTATION
		float angle_traveled = gyroscope.GetAngle() - angleAtImage;
		float angle_remaining = angleToTurn - angle_traveled;
		float angle_percent_remaining = angle_remaining / angleToTurn;
		//turn left fast
		if (angle_remaining < -13.0)
		{
			jaguarFrontLeft.Set(-(FAST_TURN * angle_percent_remaining));
			jaguarFrontRight.Set(-(FAST_TURN * angle_percent_remaining));	
			error Add the rear motors, dont fix the error until you have.;
		}
		//turn left slowly
		else if (angle_remaining < -3.0)
		{
			jaguarFrontLeft.Set(-(SLOW_TURN * angle_percent_remaining));
			jaguarFrontRight.Set(-(SLOW_TURN * angle_percent_remaining));
		}
		else if (angle_remaining < -0.5)
		{
			jaguarFrontLeft.Set(-(VERY_SLOW_TURN * angle_percent_remaining));
			jaguarFrontRight.Set(-(VERY_SLOW_TURN * angle_percent_remaining));
		}
		//turn right fast
		else if (angle_remaining > 13.0)
		{
			jaguarFrontLeft.Set((FAST_TURN * angle_percent_remaining));
			jaguarFrontRight.Set((FAST_TURN * angle_percent_remaining));
		}
		//turn right slowly
		else if (angle_remaining > 3.0)
		{
			jaguarFrontLeft.Set((SLOW_TURN * angle_percent_remaining));
			jaguarFrontRight.Set((SLOW_TURN * angle_percent_remaining));
		}
		else if (angle_remaining > 0.5)
		{
			jaguarFrontLeft.Set((VERY_SLOW_TURN * angle_percent_remaining));
			jaguarFrontRight.Set((VERY_SLOW_TURN * angle_percent_remaining));
		}
		else
		{
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
		}
#endif
	}

	void PositionForTarget(void)
	{
		typedef enum
		{
			TARGETING_IDLE,
			TARGETING_ROTATING,
			TARGETING_DRIVING_TO_DISTANCE,
		}TARGETING_STATE;
		
		static TARGETING_STATE state_targeting = TARGETING_IDLE;
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Angle Turned: %f", gyroscope.GetAngle());
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "Range to target: %f", rangeFinder.GetRangeInches());
		driverStationLCD->UpdateLCD();
		
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		switch (state_targeting)
		{
		case TARGETING_IDLE:
			rotationPID.Disable();
			rangePID.Disable();
			if (camera.IsFreshImage())
			{
				if (BUTTON_CAMERA_ALIGN_SHOT_BUTTON() && (anglesComputed == false))
				{
					ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
					camera.GetImage(colorImage);
					gyroscope.Reset();
					if (DetermineTargetPosition(colorImage) == 0)
					{
						state_targeting = TARGETING_ROTATING;
					}
					delete colorImage;
				}
			}
			break;
		case TARGETING_ROTATING:
			if (BUTTON_CAMERA_ALIGN_SHOT_BUTTON() == false)
			{
				rotationPID.Disable();
				state_targeting = TARGETING_IDLE;
			}
			else if (rotationPID.OnTarget())
			{
				rotationPID.Disable();
				state_targeting = TARGETING_DRIVING_TO_DISTANCE;
			}
			else
			{
				rotationPID.Enable();
			}
			break;
		case TARGETING_DRIVING_TO_DISTANCE:
			if (BUTTON_CAMERA_ALIGN_SHOT_BUTTON() == false)
			{
				rangePID.Disable();
				state_targeting = TARGETING_IDLE;
			}
			else if (rotationPID.OnTarget())
			{
				rangePID.Disable();
				state_targeting = TARGETING_ROTATING;
			}
			else
			{
				rangePID.Enable();
			}
			break;
			
		}
	}
	
	void ManageAppendages(void)
	{
		typedef enum
		{
			APPENDAGE_IDLE,
			APPENDAGE_BRIDGE_RAM_EXTENDED,
			APPENDAGE_BRIDGE_RAM_RETRACTING,
			APPENDAGE_DUMPER_RAMP_EXTENDED,
			APPENDAGE_DUMPER_RAMP_RETRACTING
		}APPENDAGE_STATE;
		
		static int retracting_appendage_counter = 0;
		static APPENDAGE_STATE appendage_state = APPENDAGE_IDLE;
		static bool trigger_released = false;
		switch(appendage_state)
		{
		default:
		case APPENDAGE_IDLE:
			BRIDGE_RAM_EXTENDED(false);
			DUMPER_RAMP_EXTENDED(false);
			if (!BUTTON_LOWER_BRIDGE_RAM() && !BUTTON_DUMPER_RAMP_EXTEND())
			{
				trigger_released = true;
			}
			
			if(trigger_released && BUTTON_LOWER_BRIDGE_RAM())
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(true);
				appendage_state = APPENDAGE_BRIDGE_RAM_EXTENDED;
			}
			else if (trigger_released && BUTTON_DUMPER_RAMP_EXTEND())
			{
				trigger_released = false;
				DUMPER_RAMP_EXTENDED(true);
				appendage_state = APPENDAGE_DUMPER_RAMP_EXTENDED;
			}
			break;
		case APPENDAGE_BRIDGE_RAM_EXTENDED:
			BRIDGE_RAM_EXTENDED(true);
			DUMPER_RAMP_EXTENDED(false);

			if (!BUTTON_LOWER_BRIDGE_RAM())
			{
				trigger_released = true;
			}
			if (trigger_released && BUTTON_LOWER_BRIDGE_RAM())
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(false);
				DUMPER_RAMP_EXTENDED(false);
				retracting_appendage_counter = 0;
				appendage_state = APPENDAGE_DUMPER_RAMP_RETRACTING;
			}
			break;
		case APPENDAGE_BRIDGE_RAM_RETRACTING:
			BRIDGE_RAM_EXTENDED(false);
			DUMPER_RAMP_EXTENDED(false);
			if (!BUTTON_LOWER_BRIDGE_RAM())
			{
				trigger_released = true;
			}
			
			retracting_appendage_counter++;
			if (retracting_appendage_counter > 10)
			{
				appendage_state = APPENDAGE_IDLE;
			}
			break;
		case APPENDAGE_DUMPER_RAMP_EXTENDED:

			BRIDGE_RAM_EXTENDED(false);
			DUMPER_RAMP_EXTENDED(true);

			if (!BUTTON_DUMPER_RAMP_EXTEND())
			{
				trigger_released = true;
			}
			if (trigger_released && BUTTON_DUMPER_RAMP_EXTEND())
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(false);
				DUMPER_RAMP_EXTENDED(false);
				retracting_appendage_counter = 0;
				appendage_state = APPENDAGE_DUMPER_RAMP_RETRACTING;
			}
			break;
		case APPENDAGE_DUMPER_RAMP_RETRACTING:
			BRIDGE_RAM_EXTENDED(false);
			DUMPER_RAMP_EXTENDED(false);
			if (!BUTTON_DUMPER_RAMP_EXTEND())
			{
				trigger_released = true;
			}
			retracting_appendage_counter++;
			if (retracting_appendage_counter > 10)
			{
				appendage_state = APPENDAGE_IDLE;
			}
			break;
		}
	}

	void ManageElevator(void)
	{
		float throttle = THROTTLE_ELEVATORS();
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Throttle: %f", throttle);
		
			
		
		if (BUTTON_ELEVATOR_BOTTOM_UP())
		{
			jaguarElevatorBottom1.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else if (BUTTON_ELEVATOR_BOTTOM_DOWN())
		{
			jaguarElevatorBottom1.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else
		{
			jaguarElevatorBottom1.Set(0.0);
			jaguarElevatorBottom2.Set(0.0);
		}
		
		if (BUTTON_ELEVATOR_TOP_UP())
		{
			jaguarElevatorTop.Set(ELEVATOR_SPEED_TOP);
		}
		else if (BUTTON_ELEVATOR_TOP_DOWN())
		{
			jaguarElevatorTop.Set(-ELEVATOR_SPEED_TOP);
		}
		else
		{
			jaguarElevatorTop.Set(0.0);
		}
		
		if (BUTTON_DUMPER_ROLLER())
		{
			jaguarDumperRoller.Set(-throttle);
		}
		else
		{
			jaguarDumperRoller.Set(0.0);
		}
	}
	
	void ManageCatapult(void)
	{
		typedef enum
		{
			CATAPULT_INITIAL,
			CATAPULT_COCKING,
			CATAPULT_WAITING_LATCH,
			CATAPULT_READY,
			CATAPULT_FIRING
		}CATAPULT_STATE;
		
		static CATAPULT_STATE catapult_state = CATAPULT_INITIAL;
		static Timer state_timer;
		state_timer.Start();
		switch (catapult_state)
		{
		case CATAPULT_INITIAL:
			CATAPULT_LATCH_EXTENDED(false);
			CATAPULT_PUSHER_EXTENDED(false);
			if (BUTTON_CATAPULT_SHOOT())
			{
				CATAPULT_PUSHER_EXTENDED(true);
				catapult_state = CATAPULT_COCKING;
				state_timer.Reset();
			}
			break;
		case CATAPULT_COCKING:
			CATAPULT_PUSHER_EXTENDED(true);
			CATAPULT_LATCH_EXTENDED(false);
			if (state_timer.Get() >= 2.0);
			{
				state_timer.Reset();
				catapult_state = CATAPULT_WAITING_LATCH;
			}
			break;
		case CATAPULT_WAITING_LATCH:
			CATAPULT_LATCH_EXTENDED(false);
			CATAPULT_PUSHER_EXTENDED(true);
			if (BUTTON_CATAPULT_LATCH())
			{
				CATAPULT_LATCH_EXTENDED(true);
				//maybe switch solonoid?
				state_timer.Reset();
				catapult_state = CATAPULT_READY;
			}
			break;
		case CATAPULT_READY:
			CATAPULT_PUSHER_EXTENDED(false);
			CATAPULT_LATCH_EXTENDED(true);
			if (BUTTON_CATAPULT_SHOOT() && (state_timer.Get() > 0.1))
			{
				if ((BUTTON_CATAPULT_FORCE_SHOOT() == true) ||
					((anglesComputed == true) && 
					 (angleToTurn < 1.0) && 
					 (angleToTurn > -1.0))) 
				{
					CATAPULT_LATCH_EXTENDED(false);
					state_timer.Reset();
					catapult_state = CATAPULT_FIRING;
				}
			}
			break;
		case CATAPULT_FIRING:
			CATAPULT_PUSHER_EXTENDED(false);
			CATAPULT_LATCH_EXTENDED(false);
			if (state_timer.Get() >= 1.0)
			{
				CATAPULT_PUSHER_EXTENDED(true);
				catapult_state = CATAPULT_COCKING;
			}
			break;
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
				//camera.WriteResolution(camera.kResolution_320x240);
				//camera.WriteWhiteBalance(camera.kWhiteBalance_FixedFlourescent1);
			}
		}
	}
	
	void FlipSolenoids(bool isFirstEna, Solenoid *sol1, Solenoid *sol2)
	{
		if ((sol1 == NULL) || (sol2 == NULL))
			return;
		
		if (isFirstEna)
		{
			sol2->Set(false);
			sol1->Set(true);
		}
		else
		{
			sol1->Set(false);
			sol2->Set(true);
		}
	}
};


START_ROBOT_CLASS(Robot2012);
