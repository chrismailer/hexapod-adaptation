/**
 * @file Hexapod.h
 * @author Ross Christopher (CHRROS005)
 * @date
 * @brief Hexapod class
 **/

#ifndef INCLUDE_HEXAPOD_H
#define INCLUDE_HEXAPOD_H

#include<stdio.h>

#include "stm32f4xx.h"
#include "diag/Trace.h"

#include "enumDefinitions.h"

#include "Accelerometer.h"
#include "Remote.h"
#include "Path.h"
#include "PID.h"
#include "Math.h"

#include "Leg.h"

#include "CommsManager.h"

/**
 * 	@brief Hexapod Class
 *
 *	@details
 *
 **/
class Hexapod{

private:

	/** @brief System clock tick used for timing of events. **/
	uint32_t utick = 0;

	/** @brief HAL library GPIO initialization structure. **/
	GPIO_InitTypeDef GPIO_InitStructure;

	 /** @brief HAL library ADC initialization structure. **/
	ADC_HandleTypeDef ADC_HandleStructure_Charger;

	/** @brief HAL library TIM2 initialization structure. **/
	TIM_HandleTypeDef TIM2_HandleStructure;
	/** @brief HAL library TIM3 initialization structure. **/
	TIM_HandleTypeDef TIM3_HandleStructure;

	/** @brief Register value of ADC temperature sensor calibration at 30 degrees C.  **/
	const uint16_t * const ADC_TEMP_3V3_30C =  reinterpret_cast<uint16_t*>(0x1FFF7A2C);
	/** @brief Register value of ADC temperature sensor calibration at 30 degrees C. **/
	const uint16_t * const ADC_TEMP_3V3_110C =  reinterpret_cast<uint16_t*>(0x1FFF7A2E);

	/**
	 * @brief  System Clock Configuration
	 *
	 * @details The system Clock is configured as follow :				<br/>
	 *            System Clock source            = PLL (HSE)			<br/>
	 *            SYSCLK(Hz)                     = 168000000 Hz			<br/>
	 *            HCLK(Hz)                       = 168000000 Hz			<br/>
	 *            AHB Prescaler                  = 1					<br/>
	 *            APB1 Prescaler                 = 4					<br/>
	 *            APB2 Prescaler                 = 2					<br/>
	 *            HSE Frequency(Hz)              = HSE_VALUE			<br/>
	 *            PLL_M                          = (HSE_VALUE/1000000u)	<br/>
	 *            PLL_N                          = 336					<br/>
	 *            PLL_P                          = 2					<br/>
	 *            PLL_Q                          = 7					<br/>
	 *            VDD(V)                         = 3.3					<br/>
	 *            Main regulator output voltage  = Scale1 mode			<br/>
	 *            Flash Latency(WS)              = 5					<br/>
	 */
	void SystemClock_Config(void);

	/**
	 * 	@brief Initialize the GPIO pins.
	 *
	 *	@details
	 **/
	bool GPIO_init(void);

	/**
	 * 	@brief Initialize the ADC.
	 *
	 *	@details
	 **/
	bool ADC_init(void);

	/**
	 * 	@brief Initialize timer 2.
	 *
	 *	@details
	 **/
	void TIM2_init(uint32_t time);

	/**
	 * 	@brief Initialize timer 3.
	 *
	 *	@details
	 **/
	void TIM3_init(uint32_t time);

	/**
	 * 	@brief Set the position bytes of all motors in all legs.
	 *
	 * 	@param pos A pointer to an array containing Point objects which hold the 3 sets of position bytes for each leg.
	 *
	 *	@details
	 **/
	void setPositionBits(Point<uint8_t> * pos);

	/**
	 * 	@brief Sets the leg offsets for software calibration.
	 *
	 *	@details
	 **/
	void setLegOffsets();

public:

	/** @brief Boolean holding current charging status.  **/
	bool charging = false;
	/** @brief Boolean holding the desired charging status.  **/
	bool shouldCharging = false;
	 /** @brief Communications manager object  **/
	CommsManager comms;

	 /** @brief Timer 3 interrupt counter. **/
	uint32_t TIM3Counter = 0;

	 /** @brief IMU PID control loop proportional gain.  **/
	double PID_Kp = 1;
	 /** @brief IMU PID control loop integral gain.  **/
	double PID_Ki = 0;
	 /** @brief IMU PID control loop differential gain.  **/
	double PID_Kd = 0.001;

	 /** @brief Pointer to a PID control loop object for pitch control.  **/
	PID * pidPitch = nullptr;
	 /** @brief Pointer to a PID control loop object for roll control.  **/
	PID * pidRoll = nullptr;
	/** @brief Pointer to a PID control loop object for yaw control.  **/
	PID * pidYaw = nullptr;

	 /** @brief The set point for the PID control for pitch.  **/
	double pitchSetPoint = 0;
	 /** @brief The set point for the PID control for roll.  **/
	double rollSetPoint = 0;
	 /** @brief The set point for the PID control for yaw.  **/
	double yawSetPoint = 0;

	 /** @brief The transmission frequency of the IMU data to the host computer.  **/
	double IMUTransFreq = 1;

	 /** @brief The pitch angle of the slope.  **/
	double slopePitch = 0;
	 /** @brief The roll angle of the slope.  **/
	double slopeRoll = 0;

	/** @brief The radial distance from the center of mass to the inner position of the foot tip during walking. **/
	double radialDistance = 250.0; 	// mm
	/** @brief The body rotation of the hexapod. **/
	double bodyTwist = 0;


	// Chris gait parameters modifications initialised for default tripod gait
	/** @brief The desired gait period. **/
	double gaitPeriod = 1.0; // seconds
	/** @brief The walking direction as an angle relative to the body frame. **/
//	double crabAngle = (30/180) * M_PI;
	/** @brief The walking direction as an angle in radians from leg 1. **/
	double direction = (30.0/180.0) * M_PI; 		// radians

	//MAP PARAMETERS

	double footVelocity = 382.695131;
	double bodyHeight = 86.536703;
	double radialDistances[6] = { 179.427313, 184.423759, 188.392467, 198.611087, 170.467187, 175.404773 };
	double angleOffsets[6] = { 1.047833, 0.131381, -0.966796, 0.288283, 0.165603, 0.573935 };
	double stepHeights[6] = { 10.258974, 191.638289, 3.320274, 196.184840, 186.990921, 138.930680 };
	double phaseOffsets[6] = { 0.898486, 0.147195, 0.845225, 0.762260, 0.262827, 0.111235 };
	double dutyFactors[6] = { 0.484614, 0.696863, 0.389259, 0.052084, 0.626722, 0.016228 };


	double gaitFrequency = 120;
	volatile double elapsedTime = 0;
	 /** @brief The distance that the hexapod has walked.  **/
	double distanceWalked = 0;

	/** @brief The speed of a leg in the x direction. **/
	double xd = 5;
	/** @brief The speed of a leg in the y direction. **/
	double yd = 5;
	/** @brief The speed of a leg in the z direction. **/
	double zd = 5;

	/** @brief The walking speed of the Hexapod **/
	double speed = 5;

	/** @brief The function command received from the remote. **/
	uint8_t function = 0;

	/** @brief The pitch angle of the body of the hexapod. **/
	double pitchAngle = 0;
	/** @brief The roll angle of the body of the hexapod. **/
	double rollAngle = 0;
	/** @brief The yaw angle of the body of the hexapod. **/
	double yawAngle = 0;

	/** @brief A position array containing a default standing position. **/
	double xyz_stand[6][3] = {{283.710,0.0,-this->bodyHeight},{141.855,-245.7,-this->bodyHeight},{-141.855,-245.7,-this->bodyHeight},{-283.710,0.0,-this->bodyHeight},{-141.855,245.7,-this->bodyHeight},{141.855,245.7,-this->bodyHeight}};

	/** @brief A pointer to an Accelerometer object. **/
	Accelerometer * accel = nullptr;
	/** @brief A pointer to a Remote object **/
	Remote * remote = nullptr;
	/** @brief A counter holding the current position in the walking path. **/
	uint8_t currentPathPoint = 0;
	/** @brief An array of pointers to Leg objects. **/
	Leg * legs[6];

	/** @brief Goes true if the Hexapod is ready to move to a new position.  **/
	bool newPosition = false;
	/** @brief State variable to prevent timers running until initialisations are complete.  **/
	bool timersReady = false;

	/** @brief STM32F4 internal temperature at startup. **/
	double startingTemperature = -9999;
	/** @brief STM32F4 current internal temperature.   **/
	double currentTemperature = -9999;

	 /** @brief Boolean holding whether or not the hexapod is lifted, ready to mount the charging base.  **/
	bool liftedForBase = false;

	 /** @brief Boolean holding whether or not the hexapod is in its low power mode.  **/
	bool lowPowerMode = false;

	 /** @brief A counter for determining if the battery voltage is actually low, or if it just spiked low once.  **/
	uint8_t lowBatteryCount = 0;

	/**
	 * @brief Read the internal temperature of the STM32F4.
	 *
	 * @details
	 *
	 * @return STM32F4 internal temperature as a double.
	 **/
	double readInternalTemp();

	/**
	 * @brief Read the current battery voltage and charging status.
	 *
	 * @details
	 *
	 * @return CHARGER struct holding battery voltage and charging status.
	 **/
	struct CHARGER readCharger();

	/**
	 * @brief Enter a low power state to conserve battery life.
	 *
	 * @details
	 *
	 * @return
	 **/
	void enterLowPowerMode(void);

	/**
	 * @brief Exit the low power state.
	 *
	 * @details
	 *
	 * @return
	 **/
	void exitLowPowerMode(void);

	/**
	 * @brief Enable reading of the battery voltage through the ADC.
	 *
	 * @details
	 *
	 * @return
	 **/
	void enableChargerReading();

	/**
	 * @brief Disable reading of the battery voltage.
	 *
	 * @details
	 *
	 * @return
	 **/
	void disableChargerReading();

	/**
	 * @brief Send the start charging command to the base.
	 *
	 * @details
	 *
	 * @return
	 **/
	void startCharging();

	/**
	 * @brief Send the stop charging command to the base.
	 *
	 * @details
	 *
	 * @return
	 **/
	void stopCharging();

	/**
	 * @brief Disable power to the motors.
	 *
	 * @details
	 *
	 * @return
	 **/
	void disableMotors();

	/**
	 * @brief Enable power to the motors.
	 *
	 * @details
	 *
	 * @return
	 **/
	void enableMotors();

	/**
	 * 	@brief Check if the hexapod is ready to move its legs to the next path point.
	 *
	 *	@details	UNUSED
	 **/
	bool readyForNextPosition(void);

	/**
	 * @brief Default constructor for the class.
	 *
	 * @details Class constructor
	 **/
	Hexapod(void);

	 /**
	  * @brief Default destructor for the class.
	  *
	  * @details Class destructor cleans up memory by deleting pointers and lists.
	  *
	  **/
	~Hexapod(void);

	/**
	 * 	@brief Get the absolute value of a number.
	 *
	 *	@details
	 *
	 *	@param val The value to get the absolute of.
	 *
	 *	@return The absolute value.
	 **/
	double Lib_abs(double val);

	/**
	 * @brief Reset any errors on all motors.
	 *
	 * @details
	 *
	 **/
	void resetAllMotors(void);

	/**
	 * @brief Software reset the STM32F4.
	 *
	 * @details
	 *
	 * @param check Confirmation of reset.
	 *
	 **/
	void fullSystemReset(bool check);

	/**
	 * 	@brief Global error handler function.
	 *
	 *	@details
	 **/
	void _Error_Handler(char * file, int line);

	/**
	 * 	@brief Sets the utick variable to the current system tick.
	 *
	 *	@details
	 **/
	void setTick(void);

	/**
	 * 	@brief Returns the difference between the utick variable and the current system tick.
	 *
	 *	@details And it then updates the utick variable to the current system tick.
	 **/
	uint32_t getTick(void);

	/**
	 * 	@brief Get the motor object based on the motor ID
	 *
	 *	@details
	 *
	 *	@param ID The ID of the motor to return.
	 *
	 *	@return A pointer to the motor specified.
	 **/
	Motor * getMotor(uint8_t ID);

	/**
	 * @brief Check if the remote config is correct and update it if it is incorrect.
	 *
	 * @details
	 **/
	void checkRemoteConfig();

	/**
	 * @brief Set the remote config to the correct values.
	 *
	 * @details
	 **/
	bool setRemoteConfig();

	/**
	 * @brief Set the PID pitch and roll set points.
	 *
	 * @details
	 *
	 * @param pitch The pitch set point.
	 * @param roll The roll set point.
	 **/
	void setPIDSetPoints(double pitch, double roll);

	/**
	 * @brief Get the leg number given a motor ID.
	 *
	 * @details
	 *
	 * @param motorID The ID of the motor.
	 *
	 * @return The leg number the specified motor belongs to.
	 **/
	uint8_t getMotorLegNum(uint8_t motorID);

	/**
	 * 	@brief Sets the roll, pitch and yaw angles of the hexapod.
	 *
	 *	@param pitch The pitch angle.
	 *	@param roll The roll angle.
	 *	@param yaw The yaw angle.
	 *
	 *	@details
	 **/
	void setOrientation(double pitch, double roll, double yaw);

	/**
	 * 	@brief Sets the position of each of the legs of the hexapod.
	 *
	 * 	@param pos A 2D array containing the goal positions of each of the legs.
	 *
	 *	@details
	 **/
	void setPosition(double pos[6][3]);

	/**
	 * @brief Sets the position of each of the legs of the hexapod.
	 *
	 * @details
	 *
	 * @param pos A pointer to an array containing the goal positions of each of the legs.
	 **/
	void setPosition(double * pos[6]);

	/**
	 * @brief Move the Hexapod into its default standing position.
	 *
	 * @details
	 *
	 **/
	void stand(void);

	/**
	 * @brief Set the pose (orientation)_ of the Hexapod.
	 *
	 * @details
	 *
	 * @param roll The roll to set the Hexapod to.
	 * @param pitch The pitch to set the Hexapod to.
	 **/
	void setPose(double roll, double pitch);

	/**
	 * @brief Rotate the Hexapod around its center.
	 *
	 * @details
	 *
	 * @param angle The angle to rotate by in radians. + angle is clockwise body rotation.
	 **/
	void twistBody(double angle);

	/**
	 * @brief Turn the Hexapod to face a different direction.
	 *
	 * @details
	 *
	 * @param angle The angle to turn by in radians. + angle is clockwise body rotation.
	 * @param mode The mode of turning, either ABSOLUATE or DEPENDENT on the body twist angle.
	 **/
	void turnBody(double angle, enum TURN_MODE mode);

	/**
	 * 	@brief Start the timer interrupts for timers 2 and 3.
	 *
	 *	@details
	 **/
	void startTimers(void);

	/**
	 * 	@brief Initialize the hexapod.
	 *
	 *	@details
	 **/
	void Initialise(void);

	/**
	 * 	@brief Return a pointer to the timer handler specified.
	 *
	 *	@param num The timer handler number to be returned.
	 *
	 *	@details
	 **/
	TIM_HandleTypeDef * getTIM(TIM_TypeDef * num);

	/**
	 * 	@brief Set the interrupt interval of timer 2.
	 *
	 *	@param time The interrupt interval.
	 *
	 *	@details
	 **/
	void setTIM2Time(uint32_t time);

	/**
	 * 	@brief Set the write enable pin to high.
	 *
	 *	@details GPIOD pin 12.
	 **/
	void UART1_Toggle_Write(void);

	/**
	 * 	@brief Set the write enable pin to low.
	 *
	 *	@details GPIOD pin 12.
	 **/
	void UART1_Toggle_Read(void);

	/**
	 * 	@brief Read the full control table of all of the motors.
	 *
	 *	@details
	 **/
	void readAllMotors(void);

	/**
	 * 	@brief Move the hexapod's legs to their current goal positions.
	 *
	 *	@details
	 **/
	void move(enum UART_TRANSMISSION_TYPE type);

	/**
	 * @brief Stop the hexapod from walking or moving.
	 *
	 * @details
	 *
	 * @return
	 **/
	void stop(void);

	/**
	 * @brief Lift the hexapod into a position ready to walk over the charging base.
	 *
	 * @details
	 *
	 * @return
	 **/
	void liftOntoBase(bool state);

	/**
	 * @brief Check if the desired position is reachable.
	 *
	 * @details
	 *
	 * @return
	 **/
	bool reachablePosition(void);

	/**
	 * 	@brief Generate the walking path of each of the legs.
	 *
	 *	@details
	 **/
	void makePath(void);

	/**
	 * 	@brief Set the walking parameters of the hexapod.
	 *
	 * 	@param radialDistance The radial distance from the center of mass to the inner position of the foot tip during walking.
	 * 	@param strideLength The stride length of each step in the walking path.
	 * 	@param stepHeight The step height of each step in the walking path.
	 *
	 *	@details
	 **/
	void setParamenters(double radialDistance, double strideLength, double stepHeight);

	/**
	 * 	@brief Send the ping command to the motor specified.
	 *
	 *	@return if the returned ID matches the pinged motor ID
	 *
	 *	@details
	 **/
	bool PingAll();

	/**
	 * @brief Write data to the registers of all motors.
	 *
	 * @details
	 *
	 * @param reg The register to write to.
	 * @param data A pointer to an array containing the data to write.
	 * @param dataLength The length of data to write.
	 *
	 **/
	void writeRegAll(enum CONTROL_TABLE reg, uint8_t * data, uint8_t dataLength);


	/**
	 * 	@brief	Reset the motor specified.
	 *
	 * 	@param ID The ID number of the motor to be reset.
	 *
	 *	@details
	 **/
	void resetMotor(uint8_t ID);

	/**
	 * 	@brief	Set the direction of the hexapod.
	 *
	 * 	@param dir The direction to be set.
	 *
	 *	@details
	 **/
	void setDirection(double dir);

	/**
	 * 	@brief Set the speed of the legs of the Hexapod.
	 *
	 * 	@param xd The speed of the leg in the x direction
	 * 	@param yd The speed of the leg in the y direction
	 * 	@param zd The speed of the leg in the z direction
	 *
	 *	@details
	 **/
	void setSpeed(double xd, double yd, double zd);

	/**
	 * @brief Set the speed of the legs in RPM of each motor.
	 *
	 * @details
	 *
	 * @param th1d Speed of motor 1.
	 *  @param th2d Speed of motor 2.
	 *   @param th3d Speed of motor 3.
	 *
	 * @return
	 **/
	void setSpeedRPM(double th1d, double th2d, double th3d);

	/**
	 * 	@brief Set the body height of the hexapod.
	 *
	 * 	@param height The body height to be set.
	 *
	 *	@details
	 **/
	void setBodyHeight(double height);

	/**
	 * 	@brief Sets the hexapod's legs to the next point in the currently generated path.
	 *
	 *	@details
	 **/
	void setNextPathPoint(void);

	/**
	 * 	@brief Update the hexapod's walking parameters to the data received by the Remote object.
	 *
	 *	@details
	 **/
	void update(void);
};

#endif
