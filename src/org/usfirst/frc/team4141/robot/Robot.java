/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4141.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Example demonstrating the Position closed-loop servo.
 * 
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the joystick to throttle the 
 * Talon manually (see Button 2 below).  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the setSensorPhase() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use button 1 to servo to target position.  
 * 
 * Button 1 - Enters Closed Loop Position mode; the target position is proportional to the
 *            value of the Y-axis of the joystick when button 1 is pressed
 * Button 2 - Drives the motor with power percentage based on the Y-axis of the joystick
 *            when button 2 is pressed 
 *
 * Tweak the PID gains accordingly. This can be done by:
 *    1. Changing the values in the calls below to talon.config_kP, talon.config_kI, and talon.config_kD
 *    2. Use the roboRIO web-based configuration tool to quickly change the gains on the fly
 *       without having to re-deploy the code.
 * 
 * Feedback:
 * 		RoboRio Log - prints the motor output % and the current error (if in closed loop)
 * 
 * 		SmartDashboard - Displays motor output, current error, and PID constants
 * 						 Note: it is preferable to display the error widget as a graph
 */

//  This project was based on the CTRE Sample Phoenix program on their website:
//		https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/PositionClosedLoop
//  The base program was modified somewhat to be more readable and to display information on the SmartDashboard

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class Robot extends IterativeRobot {

	int speedControllerID = 1;								// ID for the Talon speed controller to use
	TalonSRX talon = new TalonSRX(speedControllerID);		
	Joystick joystick = new Joystick(0);
	StringBuilder stringBuffer = new StringBuilder();		// Buffer to hold string to output to RioLog
	int nLoops = 0;											// Keep track of # loops called during TeleOp
	boolean wasPrevPressButton1 = false;					// Keep track of whether previous button was Button 1
	double targetPositionRotations;							// Target # rotations for closed loop mode

	public void robotInit() {

		// Choose the sensor and sensor direction
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		// Choose to ensure sensor is positive when output is positive
		talon.setSensorPhase(Constants.kSensorPhase);

		// Choose based on what direction you want forward/positive to be.
		// This does not affect sensor phase.
		talon.setInverted(Constants.kMotorInvert);

		// Set the peak and nominal outputs, 12V means full
		talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		// Set closed loop gains in slot0, typically kF stays zero.
		talon.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		talon.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		talon.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

		// Lets grab the 360 degree position of the MagEncoder's absolute
		// position, and initially set the relative sensor to match.
		int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();
		// Mask out overflows, keep bottom 12 bits
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase)	absolutePosition *= -1;
		if (Constants.kMotorInvert)	absolutePosition *= -1;
		
		// Set the quadrature (relative) sensor to match absolute
		talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	
	public void disabledPeriodic() {
		commonLoop();
	}

	// This function is called periodically during operator control
	public void teleopPeriodic() {
		commonLoop();
	}

	// This is the main loop function; called every 20ms or so
	void commonLoop() {
		
		// Get joystick Y axis value
		double yAxisValue = joystick.getY();
		
		double motorOutput = talon.getMotorOutputPercent();		// Get current motor level to display on SmartDashboard
		
		// Check whether button 1 or 2 is pressed
		boolean button1Pressed = joystick.getRawButton(1);
		boolean button2Pressed = joystick.getRawButton(2);
		
		// Deadband - Ignore Y-axis near the neutral point
		if (Math.abs(yAxisValue) < 0.10) {
			/* within 10% of zero */
			yAxisValue = 0;
		}

		// Display various status information to SmartDashboard
		SmartDashboard.putNumber("Joystick y Axis Value", yAxisValue);
		SmartDashboard.putNumber("Motor Output", motorOutput);
		SmartDashboard.putBoolean("Button1 Pressed?", button1Pressed);
		SmartDashboard.putBoolean("Button2 Pressed?", button2Pressed);
		SmartDashboard.putBoolean("Was Previous Press Button1?", wasPrevPressButton1);
		
		// Prepare line to print to RioLog
		stringBuffer.append("\tMotor Output: ");
		// Cast to int to remove decimal places to display as %
		stringBuffer.append((int) (motorOutput * 100));
		stringBuffer.append("%"); 

		stringBuffer.append("\tCurrent Position: ");
		stringBuffer.append(talon.getSelectedSensorPosition(0));
		stringBuffer.append("units");

		// On button1 press enter closed-loop mode on target position (but ignore subsequent presses)
		if (!wasPrevPressButton1 && button1Pressed) {
			// Position mode - button1 just pressed - display it in RioLog
			stringBuffer.append("Button 1 pressed\n");
			
			// Compute the target position = 10 Rotations * 4096 units/rev in either direction
			targetPositionRotations = yAxisValue * 10.0 * 4096;
			talon.set(ControlMode.Position, targetPositionRotations);
			
			// Display the target position on the SmartDashboard
			SmartDashboard.putNumber("targetPositionRotations", targetPositionRotations);
		}
		
		// On button2 just straight drive
		if (button2Pressed) {
			// Percent voltage mode - button2 just pressed - display it in RioLog
			stringBuffer.append("Button 2 pressed\n");
			
			// Set motor output to be whatever the joystick y-axis value was when button 2 pressed
			talon.set(ControlMode.PercentOutput, yAxisValue);			
		}
		
		// If Talon is in position closed-loop, print some more info
		if (talon.getControlMode() == ControlMode.Position) {
			// Output error and target position to RioLog

			stringBuffer.append("\tClosed Loop Error: ");		
			int currentClosedLoopError = talon.getClosedLoopError(Constants.kPIDLoopIdx);
			stringBuffer.append(currentClosedLoopError);
			stringBuffer.append("units");

			stringBuffer.append("\tTarget: ");
			stringBuffer.append(targetPositionRotations);
			stringBuffer.append("units"); /* units */

			// Display error in SmartDashboard
			SmartDashboard.putNumber("Current Closed Loop Error", currentClosedLoopError);
		}

		// Print every ten loops, printing too much too fast is generally bad for performance
		if (++nLoops >= 10) {
			nLoops = 0;
			System.out.println(stringBuffer.toString());
		}
		
		// Clear the RioLog string
		stringBuffer.setLength(0);
		
		// Remember the whether the previous button was button 1
		wasPrevPressButton1 = button1Pressed;
	}
}