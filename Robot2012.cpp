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




#define HALF_HORIZONTAL_FIELD_OF_VIEW 24.40828F
#define HORIZONTAL_DEGREES_PER_PIXEL (HALF_HORIZONTAL_FIELD_OF_VIEW/CENTER_OF_IMAGE)


//#define HALF_VERTICAL_FIELD_OF_VIEW 19.13711F
//#define TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW .347007F
//#define TARGET_HEIGHT_IN_FEET 1.5F
//#define IMAGE_HEIGHT_IN_PIXELS 240
//#define IMAGE_HEIGHT_IN_FEET(target_height_pxls) ((TARGET_HEIGHT_IN_FEET * (IMAGE_HEIGHT_IN_PIXELS / 2)) / target_height_pxls)
//#define CALCULATE_DISTANCE(target_height_pxls) (IMAGE_HEIGHT_IN_FEET(target_height_pxls)/TANGENT_OF_HALF_VERTICAL_FIELD_OF_VIEW)


//Controls defines - for new buttons, add a #define here and use it to get the key you want, that way we can change controls easily
#define BUTTON_CAMERA_ALIGN_SHOT_BUTTON() stickRightDrive.GetTrigger()
#define BUTTON_CAMERA_TAKE_DEBUG_PICTURES() stickRightDrive.GetRawButton(3)

#define BUTTON_ELEVATOR_BOTTOM_UP() (stickShooter.GetRawButton(2) || stickRightDrive.GetTop())
#define BUTTON_ELEVATOR_BOTTOM_DOWN() stickShooter.GetRawButton(3)
#define BUTTON_ELEVATOR_TOP_UP() stickShooter.GetRawButton(4)
#define BUTTON_ELEVATOR_TOP_DOWN() stickShooter.GetRawButton(5)

#define BUTTON_CATAPULT_SHOOT() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_LATCH() stickShooter.GetTrigger()
#define BUTTON_CATAPULT_FORCE_SHOOT() stickShooter.GetRawButton(10)
#define THROTTLE_ELEVATORS() (1.0)

#define BUTTON_DUMPER_RAMP_EXTEND() stickShooter.GetRawButton(9)
#define BUTTON_DUMPER_ROLLER() stickShooter.GetRawButton(11)

#define BUTTON_LOWER_BRIDGE_RAM() stickLeftDrive.GetTop()

//Kinect defines
#define KINECT_HEAD_RIGHT() kinectLeft.GetRawButton(1)
#define KINECT_HEAD_LEFT() kinectLeft.GetRawButton(2)
#define KINECT_RIGHT_LEG_RIGHT() kinectLeft.GetRawButton(3)
#define KINECT_LEFT_LEG_LEFT() kinectLeft.GetRawButton(4)
#define KINECT_RIGHT_LEG_FORWARD() kinectLeft.GetRawButton(5)
#define KINECT_RIGHT_LEG_BACK() kinectLeft.GetRawButton(6)
#define KINECT_LEFT_LEG_FORWARD() kinectLeft.GetRawButton(7)
#define KINECT_LEFT_LEG_BACK() kinectLeft.GetRawButton(8)
#define KINECT_CONTROL_ENABLED() kinectLeft.GetRawButton(9)

#define KINECT_ELEVATORS_UP() KINECT_RIGHT_LEG_RIGHT()
#define KINECT_ELEVATORS_DOWN() KINECT_LEFT_LEG_LEFT()
#define KINECT_AUTONOMOUS_SHOOT() KINECT_HEAD_RIGHT()
#define KINECT_BRIDGE_RAM_EXTEND() KINECT_HEAD_LEFT()




//PID Parameters
#define ROTATION_PID_PROPORTION 0.18
#define ROTATION_PID_INTEGRAL 0.03
#define ROTATION_PID_DERIVATIVE 0.07

#define ROTATION_PID_MIN_INPUT -30.0
#define ROTATION_PID_MAX_INPUT 30.0
#define ROTATION_PID_MIN_OUTPUT -1.00
#define ROTATION_PID_MAX_OUTPUT 1.00

#define ROTATION_PID_TOLERENCE_FIRST 2.50
#define ROTATION_PID_TOLERENCE_LAST 0.50
#define ROTATION_PID_SETPOINT_OFFSET -3.9 //negative adjusts to the right

#define RANGE_PID_PROPORTION 0.02
#define RANGE_PID_INTEGRAL 0.0005
#define RANGE_PID_DERIVATIVE 0.005

#define RANGE_PID_MIN_INPUT 0.0
#define RANGE_PID_MAX_INPUT 240.0
#define RANGE_PID_MIN_OUTPUT -0.40
#define RANGE_PID_MAX_OUTPUT 0.40

#define RANGE_PID_SETPOINT 150.0
#define RANGE_PID_TOLERENCE 4.0

//Comment this out to allow range adjusting in autonomous
#define DISABLE_RANGE_ADJUST_IN_AUTONOMOUS
//#define ENABLE_FOUR_SHOT_SUPER_AUTONOMOUS
#define DISABLE_RANGE_FINDER
#define LINING_UP_IN_AUTONOMOUS false


#define AUTONOMOUS_BACKUP_TIME 4.4
//Uncomment this to enable the PID tuning section of code that can help tune PIDs
//by running code in debug mode and using breakpoints.
//#define PID_TUNING

//Helper Macros
#define DUMPER_RAMP_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidDumperRampUp, &solenoidDumperRampDown);}
#define BRIDGE_RAM_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidBridgeRamDown, &solenoidBridgeRamUp);}
#define CATAPULT_PUSHER_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultPusher, &solenoidCatapultPuller);}
#define CATAPULT_LATCH_EXTENDED(isExtended) {FlipSolenoids(isExtended, &solenoidCatapultLatchExtend, &solenoidCatapultLatchRetract);}
#define DISABLE_PID(pid) {if (pid.IsEnabled()){pid.Disable();pid.Reset();}}
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
	Victor victorDumperRoller;
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
	KinectStick kinectLeft;
	KinectStick kinectRight;
	Gyro gyroscope;
	GyroControlledTurning rotationControl;
	PIDController rotationPID;
	AnalogRangeFinder rangeFinder;
	SonarControlledDriving sonarDriveControl;
	PIDController rangePID;
	Compressor compressor;

	Timer timeInState;
	Timer timeSinceBoot;
	Timer autonomousTempTimer;
	
	DriverStation *driverStation;
	DriverStationLCD *driverStationLCD;
		
		// Local variables to count the number of periodic loops performed
	
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	bool cameraInitialized;
	float angleToTurn;
	float angleAtImage;
	float distanceToTarget;
	bool targetLocked;
	int autonomousShotsFired;
	typedef enum 
	{
		AUTONOMOUS_LINING_UP_SHOT,
		AUTONOMOUS_SHOOTING_FIRST_SHOT,
		AUTONOMOUS_REARMING_FIRST_SHOT,
		AUTONOMOUS_RELOADING,
		AUTONOMOUS_SHOOTING_SECOND_SHOT,
#ifdef ENABLE_FOUR_SHOT_SUPER_AUTONOMOUS		
		AUTONOMOUS_REARMING_SECOND_SHOT,
		AUTONOMOUS_RELOADING_FOR_THIRD,
		AUTONOMOUS_SHOOTING_THIRD_SHOT,
		AUTONOMOUS_REARMING_THIRD_SHOT,
		AUTONOMOUS_RELOADING_FOR_FOURTH,
		AUTONOMOUS_SHOOTING_FOURTH_SHOT,
#else
		AUTONOMOUS_HITTING_BRIDGE,
		AUTONOMOUS_WAIT_FOR_TELEOP,
#endif
		AUTONOMOUS_DONE
	}AUTONOMOUS_STATE;
	
	AUTONOMOUS_STATE autonomousState;
		
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
		victorDumperRoller(DIGITAL_OUTPUT_CHANNEL, PWM_CHANNEL_8_JAGUAR_DUMPER_ROLLER),
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
		kinectLeft(1),
		kinectRight(2),
		gyroscope(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_1_GYROSCOPE),
		rotationControl(&myRobot),
		rotationPID(ROTATION_PID_PROPORTION, ROTATION_PID_INTEGRAL, ROTATION_PID_DERIVATIVE, &gyroscope, &rotationControl),
		rangeFinder(ANALOG_OUTPUT_CHANNEL, ANALOG_CHANNEL_2_RANGE_FINDER),
		sonarDriveControl(&myRobot),
		rangePID(RANGE_PID_PROPORTION,RANGE_PID_INTEGRAL,RANGE_PID_DERIVATIVE, &rangeFinder, &sonarDriveControl),
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
		rotationPID.SetInputRange(ROTATION_PID_MIN_INPUT,ROTATION_PID_MAX_INPUT);
		rotationPID.SetOutputRange(ROTATION_PID_MIN_OUTPUT,ROTATION_PID_MAX_OUTPUT);
		rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_FIRST);
		rotationPID.Disable();
		
		rangePID.SetInputRange(RANGE_PID_MIN_INPUT,RANGE_PID_MAX_INPUT);
		rangePID.SetOutputRange(RANGE_PID_MIN_OUTPUT, RANGE_PID_MAX_OUTPUT);
		rangePID.SetSetpoint(RANGE_PID_SETPOINT);
		rangePID.SetTolerance(RANGE_PID_TOLERENCE);
		rangePID.Disable();
		
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
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Stop();
		myRobot.SetSafetyEnabled(false);
	}

	void AutonomousInit(void) 
	{
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		autonomousState = AUTONOMOUS_LINING_UP_SHOT;
		autonomousTempTimer.Reset();
		autonomousTempTimer.Start();
		myRobot.SetSafetyEnabled(true);
	}

	void TeleopInit(void) 
	{
		m_autoPeriodicLoops = 0;				
		m_telePeriodicLoops = 0;				
		m_disabledPeriodicLoops = 0;			
		timeInState.Reset();
		timeInState.Start();
		compressor.Start();
		myRobot.SetSafetyEnabled(true);
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  
	{
		m_disabledPeriodicLoops++;
	}

	void AutonomousPeriodic(void) 
	{
		static bool useKinect = false;
		static bool kinectAutonomous = false;
		static Kinect *kinect = Kinect::GetInstance();
		if (m_autoPeriodicLoops == 0)
		{
			useKinect = false;
			kinectAutonomous = false;
		}
		m_autoPeriodicLoops++;
		CameraInitialize();

		
		#ifdef ENABLE_FOUR_SHOT_SUPER_AUTONOMOUS
		autonomousTempTimer.Start(); //we call this to make sure the timer stays running whenever we are autoshooting.
			
			//Depending on the current autonomousState, we run one of the following cases (this is called a switch case statement)
			switch (autonomousState)
			{
			//Cumulative time: 0.0
			case AUTONOMOUS_LINING_UP_SHOT:
				//in this state set all the managers to off accept the targeter
				ManageAppendages(false,false);
				ManageElevator(false,false,false,false,false,0.5);
				ManageCatapult(false, false, false);
				//if you hover over the function name (ie ManageAppendages) you can see what parameters it takes, and determine what they do by their name
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				
				
				//if targetLocked or autonTempTimer > 5sec goto the next state
				if ((autonomousTempTimer.Get() > 2.0))
				{
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_SHOOTING_FIRST_SHOT;
				}
				break;
			//Cumulative time: 2.0
			case AUTONOMOUS_SHOOTING_FIRST_SHOT:
				ManageAppendages(false,false);
				ManageElevator(false,false,false,false,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(true, false, true);
				autonomousState = AUTONOMOUS_REARMING_FIRST_SHOT;
				break;
			//Cumulative time: 2.01
			case AUTONOMOUS_REARMING_FIRST_SHOT:
				ManageAppendages(false,false);
				ManageElevator(false,false,false,false,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, false, false);
				if (autonomousTempTimer.Get() > 1.0)
				{
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_RELOADING;
				}
				break;
			//Cumulative time: 3.0
			case AUTONOMOUS_RELOADING:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, true, false);
				if (autonomousTempTimer.Get()> 3.0)
				{
					ManageCatapult(false, false, false);
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_SHOOTING_SECOND_SHOT;
				}
				break;
			//Cumulative time: 6.0
			case AUTONOMOUS_SHOOTING_SECOND_SHOT:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				//if target_locked is true (or time > 1.0) push the catapult fire (and force_shoot) and wait 2 second before going to AUTONOMOUS_DONE
				ManageCatapult(true, false, true);
				autonomousState = AUTONOMOUS_REARMING_SECOND_SHOT;
				break;
			//Cumulative time: 6.01
			case AUTONOMOUS_REARMING_SECOND_SHOT:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, false, false);
				if (autonomousTempTimer.Get() > 1.0)
				{
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_RELOADING;
				}
				break;
			//Cumulative time: 7.0
			case AUTONOMOUS_RELOADING_FOR_THIRD:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, true, false);
				if (autonomousTempTimer.Get()> 3.0)
				{
					ManageCatapult(false, false, false);
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_SHOOTING_SECOND_SHOT;
				}
				break;
			//Cumulative time: 10.0
			case AUTONOMOUS_SHOOTING_THIRD_SHOT:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				//if target_locked is true (or time > 1.0) push the catapult fire (and force_shoot) and wait 2 second before going to AUTONOMOUS_DONE
				ManageCatapult(true, false, true);
				autonomousState = AUTONOMOUS_REARMING_THIRD_SHOT;
				break;
			//Cumulative time: 10.01
			case AUTONOMOUS_REARMING_THIRD_SHOT:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, false, false);
				if (autonomousTempTimer.Get() > 1.0)
				{
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_RELOADING;
				}
				break;
			//Cumulative time: 11.0
			case AUTONOMOUS_RELOADING_FOR_FOURTH:
				ManageAppendages(false,false);
				ManageElevator(true,false,false,true,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(false, true, false);
				if (autonomousTempTimer.Get()> 3.0)
				{
					ManageCatapult(false, false, false);
					autonomousTempTimer.Reset();
					autonomousState = AUTONOMOUS_SHOOTING_SECOND_SHOT;
				}
				break;
			//Cumulative time 14.0
			case AUTONOMOUS_SHOOTING_FOURTH_SHOT:
				ManageAppendages(false,false);
				ManageElevator(false,false,false,false,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(true, false, true);
				autonomousState = AUTONOMOUS_DONE;
				break;
			case AUTONOMOUS_DONE:
				ManageAppendages(false,false);
				ManageElevator(false,false,false,false,false,0.5);
				PositionForTarget(LINING_UP_IN_AUTONOMOUS);
				ManageCatapult(true, false, true);
				break;
			}
		#else
			//if there is a kinect present, and it sees somebody, use the kinect from here on in
			if (kinect != (void *) 0)
			{
				if (kinect->GetNumberOfPlayers() != 0)
				{
					useKinect = true;
				}
			}

			if ((useKinect == true) && (KINECT_CONTROL_ENABLED() == true))
			{
				kinectAutonomous = KINECT_AUTONOMOUS_SHOOT();
			}
			
			//if the kinect isnt being used or the kinect autoshoot button is button is being pressed, and we haven't finished shooting
			if (((useKinect == false) || (kinectAutonomous == true)) && (autonomousState != AUTONOMOUS_DONE))
			{
				autonomousTempTimer.Start(); //we call this to make sure the timer stays running whenever we are autoshooting.
				
				//Depending on the current autonomousState, we run one of the following cases (this is called a switch case statement)
				switch (autonomousState)
				{
				case AUTONOMOUS_LINING_UP_SHOT:
					//in this state set all the managers to off accept the targeter
	
					ManageAppendages(false,false);
					ManageElevator(false,false,false,false,false,0.5);
					ManageCatapult(false, false, false);
					//if you hover over the function name (ie ManageAppendages) you can see what parameters it takes, and determine what they do by their name
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					
					
					//if targetLocked or autonTempTimer > 5sec goto the next state
					if ((targetLocked == true) || (autonomousTempTimer.Get() > 5.0) || (LINING_UP_IN_AUTONOMOUS == false))
					{
						autonomousTempTimer.Reset();
						autonomousState = AUTONOMOUS_SHOOTING_FIRST_SHOT;
					}
					break;
				case AUTONOMOUS_SHOOTING_FIRST_SHOT:
					ManageAppendages(false,false);
					ManageElevator(false,false,false,false,false,0.5);
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					//if target_locked is true (or time > 1.0) push the catapult fire (and force_shoot) and reset the autonomousTempTimer
					if ((targetLocked == true) || (autonomousTempTimer.Get() > 1.0) || (LINING_UP_IN_AUTONOMOUS == false))
					{
						ManageCatapult(true, false, true);
						autonomousTempTimer.Reset();
						autonomousState = AUTONOMOUS_REARMING_FIRST_SHOT;
					}
					else
					{
						ManageCatapult(false, false, false);
					}
					
					break;
				case AUTONOMOUS_REARMING_FIRST_SHOT:
					//then let go of the latch button and wait 2 second before going to AUTONOMOUS_RELOADING
					ManageAppendages(false,false);
					ManageElevator(false,false,false,false,false,0.5);
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					ManageCatapult(false, false, false);
					if (autonomousTempTimer.Get() > 2.0)
					{
						autonomousTempTimer.Reset();
						autonomousState = AUTONOMOUS_RELOADING;
					}
					break;
				case AUTONOMOUS_RELOADING:
					ManageAppendages(false,false);
					ManageElevator(false,false,false,true,false,0.5);
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					ManageCatapult(false, true, false);
					if (autonomousTempTimer.Get() > 4.0)
					{
						ManageCatapult(false, false, false);
						autonomousTempTimer.Reset();
						autonomousState = AUTONOMOUS_SHOOTING_SECOND_SHOT;
					}
					//push the latch button to lock the arm in place and retract the pusher
					//turn on the top elevator (downward) for 4 seconds to reload the ball
					break;
				case AUTONOMOUS_SHOOTING_SECOND_SHOT:
					ManageAppendages(false,false);
					ManageElevator(false,false,false,true,false,0.5);
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					//if target_locked is true (or time > 1.0) push the catapult fire (and force_shoot) and wait 2 second before going to AUTONOMOUS_DONE
					if ((targetLocked == true) || (autonomousTempTimer.Get() > 1.0))
					{
						ManageCatapult(true, false, true);
						autonomousTempTimer.Reset();
						if (useKinect == true)
						{
							autonomousState = AUTONOMOUS_DONE;
						}
						else
						{
							autonomousState = AUTONOMOUS_HITTING_BRIDGE;
						}
					}
					else
					{
						ManageCatapult(false, false, false);
					}
					break;
				case AUTONOMOUS_HITTING_BRIDGE:
					//ManageAppendages(true,false); //this complicates it more than neccessary, just control the bridge ram manually
					BRIDGE_RAM_EXTENDED(true);
					ManageElevator(true,false,false,false,false,0.5);
					PositionForTarget(false);
					ManageCatapult(false, false, false);
					if (autonomousTempTimer.Get() > AUTONOMOUS_BACKUP_TIME)
					{
						autonomousTempTimer.Reset();
						myRobot.TankDrive(0.0,0.0);
						autonomousState = AUTONOMOUS_WAIT_FOR_TELEOP;
					}
					else if (autonomousTempTimer.Get() > 0.5)
					{
						myRobot.TankDrive(-.5,-.5);
					}
					else
					{
						myRobot.TankDrive(0.0,0.0);
					}
					break;
				case AUTONOMOUS_WAIT_FOR_TELEOP:
					//ManageAppendages(false,false);
					if (autonomousTempTimer.Get() < 2.0)
					{
						BRIDGE_RAM_EXTENDED(true);
					}
					else
					{
						BRIDGE_RAM_EXTENDED(false);
					}
					ManageElevator(true,false,false,false,false,0.5);
					PositionForTarget(false);
					ManageCatapult(false, false, false);
					myRobot.TankDrive(0.0,0.0);
					break;
				default:
				case AUTONOMOUS_DONE:
					ManageAppendages(false,false);
					ManageElevator(false,false,false,false,false,0.5);
					PositionForTarget(LINING_UP_IN_AUTONOMOUS);
					ManageCatapult(false, false, false);
					break;
				}
			}
			else if (KINECT_CONTROL_ENABLED() == true)
			{
				//TODO evaluate this logic.  It may not be neccessary.
				//if the kinect autoshoot button is no longer being pressed or we were already done shooting
				if ((autonomousState != AUTONOMOUS_DONE) || (KINECT_AUTONOMOUS_SHOOT()==false))
				{
					autonomousState = AUTONOMOUS_LINING_UP_SHOT; //reset the autonomous state if the kinect takes control;
				}
				autonomousTempTimer.Stop();
				autonomousTempTimer.Reset();
				myRobot.TankDrive(kinectLeft.GetY() * 0.7, kinectRight.GetY() * 0.7);
				//add code to bind each kinectStick button to each action we want to be able to do in autonomous
				//ManageAppendages(KINECT_RIGHT_LEG_BACK(),false);
				BRIDGE_RAM_EXTENDED(KINECT_BRIDGE_RAM_EXTEND());
				ManageElevator((KINECT_ELEVATORS_UP() || KINECT_BRIDGE_RAM_EXTEND()),KINECT_ELEVATORS_DOWN(),
						KINECT_ELEVATORS_UP(),KINECT_ELEVATORS_DOWN(),
						KINECT_ELEVATORS_UP(),1.0);
				PositionForTarget(false);
				ManageCatapult(false, false, false);
				
			}
		#endif
		
		
	}

	
	void TeleopPeriodic(void) 
	{
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		
		if (!BUTTON_CAMERA_ALIGN_SHOT_BUTTON())
		{
			DISABLE_PID(rotationPID);
			DISABLE_PID(rangePID);
			myRobot.TankDrive(stickLeftDrive,stickRightDrive);	
			myRobot.SetSafetyEnabled(true);
		}
		CameraInitialize();

		
		ManageAppendages(BUTTON_LOWER_BRIDGE_RAM(),BUTTON_DUMPER_RAMP_EXTEND());
		ManageElevator(BUTTON_ELEVATOR_BOTTOM_UP(), BUTTON_ELEVATOR_BOTTOM_DOWN(), 
						BUTTON_ELEVATOR_TOP_UP(), BUTTON_ELEVATOR_TOP_DOWN(), 
						BUTTON_DUMPER_ROLLER(), THROTTLE_ELEVATORS());
		ManageCatapult(BUTTON_CATAPULT_SHOOT(), BUTTON_CATAPULT_LATCH(), BUTTON_CATAPULT_FORCE_SHOOT());

		PositionForTarget(BUTTON_CAMERA_ALIGN_SHOT_BUTTON());
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
	}

	//returns 0 if a particle is found
	int DetermineTargetPosition(ColorImage *colorImage)
	{
		int returnVal = -1;
		BinaryImage *binaryImage;
		Image *imaqImage;
		if ((colorImage == (void *) 0) || (colorImage->GetWidth() == 0) || (colorImage->GetHeight() == 0))
		{
			return returnVal;
		}
		if (BUTTON_CAMERA_TAKE_DEBUG_PICTURES())
		{
			colorImage->Write("capturedImage.jpg");
		}
		
		//binaryImage = colorImage->ThresholdHSV(0, 255, 0, 255, 255, 0);
		//binaryImage = colorImage->ThresholdHSV(56, 125, 55, 255, 255, 150);
		
		binaryImage = colorImage->ThresholdHSL(90, 115,30, 255, 70, 255);
		
		if ((binaryImage == (void *) 0) || (binaryImage->GetWidth() == 0) || (binaryImage->GetHeight() == 0))
			return returnVal;
		
		imaqImage = binaryImage->GetImaqImage();
		
		if (imaqImage == (void *) 0)
			return returnVal;
		
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
		
		for (int particleIndex = 0; particleIndex < reports->size(); particleIndex++)
		{
			ParticleAnalysisReport &thisReport = reports->at(particleIndex);
			//TODO Validate the particle so we don't end up targeting a spec on the wall? (particle filter gets rid fo really small stuff, but we may want this if the arena is noisy)
			if ((!topParticlePtr) || (thisReport.center_mass_y  < topParticlePtr->center_mass_y)  /*&& thisReport.particleQuality > WHATT!!!!*/)
			{
				topParticlePtr = &thisReport;
			}
		}
		if (topParticlePtr != NULL)
		{
			returnVal = 0; //indicates success
			int centerOfMassX = topParticlePtr->center_mass_x;
			
			angleToTurn = (centerOfMassX - CENTER_OF_IMAGE) * HORIZONTAL_DEGREES_PER_PIXEL;
			
			//distanceToTarget = CALCULATE_DISTANCE(topParticlePtr->boundingRect.height);

			driverStationLCD->PrintfLine((DriverStationLCD::Line) 0, "Angle to turn: %.5f", -angleToTurn);
			//driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Distance: %.5f", distanceToTarget);
			driverStationLCD->UpdateLCD();

			#ifdef PID_TUNING
				static float tol_angle = ROTATION_PID_TOLERENCE;
				static float set_angle = ROTATION_PID_SETPOINT_OFFSET;
				float p = rotationPID.GetP();
				float i = rotationPID.GetI();
				float d = rotationPID.GetD();
				rotationPID.SetPID(p,i,d);
				
				rotationPID.SetTolerance(tol_angle);
				rotationPID.SetSetpoint((-angleToTurn) + set_angle);
				
				static float tol_range = RANGE_PID_TOLERENCE;
				static float set_range = RANGE_PID_SETPOINT;
				p = rangePID.GetP();
				i = rangePID.GetI();
				d = rangePID.GetD();
				rangePID.SetPID(p,i,d);
				rangePID.SetTolerance(tol_range);
				rangePID.SetSetpoint(set_range);
			#else
			rotationPID.SetSetpoint((-angleToTurn) + ROTATION_PID_SETPOINT_OFFSET);
			#endif
		}	
		else
		{	
			jaguarFrontLeft.Set(0.0);
			jaguarFrontRight.Set(0.0);
			jaguarRearLeft.Set(0.0);
			jaguarRearRight.Set(0.0);
		}
		delete reports;
		delete binaryImage;

		return returnVal;
	}
	

	void ManageAppendages(bool extend_bridge_ram, bool extend_dumper_ramp)
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
			if (!extend_bridge_ram && !extend_dumper_ramp)
			{
				trigger_released = true;
			}
			
			if(trigger_released && extend_bridge_ram)
			{
				trigger_released = false;
				BRIDGE_RAM_EXTENDED(true);
				appendage_state = APPENDAGE_BRIDGE_RAM_EXTENDED;
			}
			else if (trigger_released && extend_dumper_ramp)
			{
				trigger_released = false;
				DUMPER_RAMP_EXTENDED(true);
				appendage_state = APPENDAGE_DUMPER_RAMP_EXTENDED;
			}
			break;
		case APPENDAGE_BRIDGE_RAM_EXTENDED:
			BRIDGE_RAM_EXTENDED(true);
			DUMPER_RAMP_EXTENDED(false);

			if (!extend_bridge_ram)
			{
				trigger_released = true;
			}
			if (trigger_released && extend_bridge_ram)
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
			if (!extend_bridge_ram)
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

			if (!extend_dumper_ramp)
			{
				trigger_released = true;
			}
			if (trigger_released && extend_dumper_ramp)
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
			if (!extend_dumper_ramp)
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

	void ManageElevator(bool bottom_up, bool bottom_down, bool top_up, bool top_down, bool dumper_roller, float throttle)
	{
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 3, "Throttle: %f", throttle);
		
			
		
		if (bottom_up)
		{
			jaguarElevatorBottom1.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(-ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else if (bottom_down)
		{
			jaguarElevatorBottom1.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
			jaguarElevatorBottom2.Set(ELEVATOR_SPEED_BOTTOM, BOTTOM_ROLLERS_SYNC_GROUP);
		}
		else
		{
			jaguarElevatorBottom1.Set(0.0);
			jaguarElevatorBottom2.Set(0.0);
		}
		
		if (top_up)
		{
			jaguarElevatorTop.Set(ELEVATOR_SPEED_TOP);
		}
		else if (top_down)
		{
			jaguarElevatorTop.Set(-ELEVATOR_SPEED_TOP);
		}
		else
		{
			jaguarElevatorTop.Set(0.0);
		}
		
		if (dumper_roller)
		{
			victorDumperRoller.Set(throttle);
		}
		else
		{
			victorDumperRoller.Set(0.0);
		}
	}
	
	void ManageCatapult(bool catapult_shoot, bool catapult_latch, bool force_shoot)
	{
		typedef enum
		{
//			CATAPULT_INITIAL,
			CATAPULT_COCKING,
			CATAPULT_WAITING_LATCH,
			CATAPULT_READY,
			CATAPULT_FIRING
		}CATAPULT_STATE;
		
		static CATAPULT_STATE catapult_state = CATAPULT_READY;
		static bool button_released = true;
		static Timer state_timer;
		state_timer.Start();//Doesn't do anything unless it's not running
		
		switch (catapult_state)
		{
//		case CATAPULT_INITIAL:
//			CATAPULT_LATCH_EXTENDED(true);
//			CATAPULT_PUSHER_EXTENDED(false);
//			if (!catapult_shoot)button_released = true;
//			//TODO Make this use a timer instead
//			if (catapult_shoot && button_released)
//			{
//				CATAPULT_PUSHER_EXTENDED(true);
//				catapult_state = CATAPULT_COCKING;
//				state_timer.Reset();
//				button_released = false;
//			}
//			break;
		case CATAPULT_COCKING:
			CATAPULT_PUSHER_EXTENDED(true);
			CATAPULT_LATCH_EXTENDED(false);
			if (!catapult_latch)button_released = true;
			if (state_timer.Get() >= 2.0);
			{
				state_timer.Reset();
				catapult_state = CATAPULT_WAITING_LATCH;
			}
			break;
		case CATAPULT_WAITING_LATCH:
			CATAPULT_LATCH_EXTENDED(false);
			CATAPULT_PUSHER_EXTENDED(true);
			if (!catapult_latch)button_released = true;
			if (catapult_latch && button_released)
			{
				CATAPULT_LATCH_EXTENDED(true);
				//maybe switch solonoid?
				state_timer.Reset();
				catapult_state = CATAPULT_READY;
				button_released = false;
			}
			break;
		case CATAPULT_READY:
			CATAPULT_PUSHER_EXTENDED(false);
			CATAPULT_LATCH_EXTENDED(true);
			if (!catapult_shoot)button_released = true;
			if (catapult_shoot && (state_timer.Get() > 0.2) && button_released)
			{
				if ((force_shoot == true) || (targetLocked == true)) 
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
			if (!catapult_shoot)button_released = true;
			if (state_timer.Get() >= 1.0)
			{
				state_timer.Reset();
				CATAPULT_PUSHER_EXTENDED(true);
				catapult_state = CATAPULT_COCKING;
			}
			break;
		}
	}

	void PositionForTarget(bool camera_align_shot)
	{
		typedef enum
		{
			TARGETING_IDLE,
			TARGETING_ROTATING,
			TARGETING_DRIVING_TO_DISTANCE,
			TARGETING_ROTATING_FINAL
		}TARGETING_STATE;
		
		static TARGETING_STATE state_targeting = TARGETING_IDLE;
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 1, "Angle Turned: %f", gyroscope.GetAngle());
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 4, "Range to target: %f", rangeFinder.GetRangeInches());
		driverStationLCD->PrintfLine((DriverStationLCD::Line) 5, "Autoposition State: %d", state_targeting);
		driverStationLCD->UpdateLCD();
		
		static int on_target_count = 0;
		Timer imageRefreshTimer;
		AxisCamera &camera = AxisCamera::GetInstance("10.24.74.11");
		switch (state_targeting)
		{
		case TARGETING_IDLE:
			targetLocked = false;

			DISABLE_PID(rotationPID);
			DISABLE_PID(rangePID);
			if (camera.IsFreshImage())
			{
				if (camera_align_shot == true)
				{
					ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
					camera.GetImage(colorImage);
					imageRefreshTimer.Reset();
					imageRefreshTimer.Start();
					gyroscope.Reset();
					if (DetermineTargetPosition(colorImage) == 0)
					{
						rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_FIRST);
						rotationPID.Enable();
						state_targeting = TARGETING_ROTATING;
						on_target_count = 0;
					}
					delete colorImage;
				}
			}
			break;
		case TARGETING_ROTATING:
			if (camera_align_shot == false)
			{
				DISABLE_PID(rotationPID);
				state_targeting = TARGETING_IDLE;
				on_target_count = 0;
			}
			else if (rotationPID.OnTarget() && (on_target_count > 3))
			{
				on_target_count = 0;
#ifdef DISABLE_RANGE_FINDER
				state_targeting = TARGETING_ROTATING_FINAL;
#else
				
				#ifdef DISABLE_RANGE_ADJUST_IN_AUTONOMOUS
				if (m_autoPeriodicLoops != 0)
				{
					state_targeting = TARGETING_ROTATING_FINAL;
				}
				else
				#endif
				{
					DISABLE_PID(rotationPID);
					rangePID.Enable();
					state_targeting = TARGETING_DRIVING_TO_DISTANCE;
				}
#endif
			}
			else if (rotationPID.OnTarget())
			{
				rotationPID.Enable();
				on_target_count++;
			}
			else
			{
				rotationPID.Enable();
				on_target_count = 0;
			}
			break;
			//TODO need to put in a small delay for autonomous
		case TARGETING_DRIVING_TO_DISTANCE:
			if (camera_align_shot == false)
			{
				DISABLE_PID(rangePID);
				state_targeting = TARGETING_IDLE;
				on_target_count = 0;
			}
			else if (rangePID.OnTarget() && (on_target_count > 8))
			{
				DISABLE_PID(rangePID);
				state_targeting = TARGETING_ROTATING_FINAL;
				rotationPID.SetTolerance(ROTATION_PID_TOLERENCE_LAST);
				on_target_count = 0;
			}
			else if (rangePID.OnTarget())
			{
				rangePID.Enable();
				on_target_count++;
			}
			else
			{
				rangePID.Enable();
				on_target_count = 0;
			}
			break;
		case TARGETING_ROTATING_FINAL:
			
			if (camera.IsFreshImage() && camera_align_shot && (imageRefreshTimer.HasPeriodPassed(1.0)))
			{
				ColorImage *colorImage = new ColorImage(IMAQ_IMAGE_HSL);
				camera.GetImage(colorImage);
				imageRefreshTimer.Reset();
				imageRefreshTimer.Start();
				if (DetermineTargetPosition(colorImage) == 0)
				{
					gyroscope.Reset();
				}
				delete colorImage;
			}
			DISABLE_PID(rangePID);
			if (camera_align_shot == false)
			{
				DISABLE_PID(rotationPID);
				state_targeting = TARGETING_IDLE;
				targetLocked = false;
				on_target_count = 0;
			}
			else if (rotationPID.OnTarget() && (on_target_count > 6))
			{
				targetLocked = true;
				rotationPID.Enable();
			}
			else if (rotationPID.OnTarget())
			{
				rotationPID.Enable();
				on_target_count++;
			}
			else
			{
				rotationPID.Enable();
				targetLocked = false;
				on_target_count = 0;
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
