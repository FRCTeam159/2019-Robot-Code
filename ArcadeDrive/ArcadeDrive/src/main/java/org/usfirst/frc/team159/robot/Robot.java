/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team159.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot implements MotorSafety {
	private Joystick stick;
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);
	 
	
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX backRight;
	
//  Motor IDs
  public static final int FRONT_LEFT = 3;
  public static final int BACK_LEFT = 4;

  public static final int FRONT_RIGHT = 2;
  public static final int BACK_RIGHT = 1;
	@Override
	public void robotInit() {
		stick = new Joystick(0);
	//  m_rightStick = new Joystick(5);
		frontLeft = new WPI_TalonSRX(FRONT_LEFT);
		frontRight = new WPI_TalonSRX(FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(BACK_LEFT);
		backRight = new WPI_TalonSRX(BACK_RIGHT);
	}

	@Override
	public void teleopPeriodic() {
		double x = stick.getRawAxis(1);
		double y = stick.getRawAxis(4);
		arcadeDrive(x,y);
		System.out.println("x = " + x + " y = " + y);
	}
	public void arcadeDrive(double moveValue, double turnValue) {
		double leftMotorOutput;
		double rightMotorOutput;
		
		if (moveValue > 0.0) {
			if (turnValue > 0.0) {
                leftMotorOutput = Math.max(moveValue, turnValue);
				rightMotorOutput = moveValue - turnValue;
			} else {
                leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = Math.max(moveValue, -turnValue);
			}
		} else {
			if (turnValue > 0.0) {
                leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = -Math.max(-moveValue, turnValue);
			} else {
                leftMotorOutput = -Math.max(-moveValue, -turnValue);
				rightMotorOutput = moveValue - turnValue;
			}
		}
//		 Make sure values are between -1 and 1
		leftMotorOutput = coerce(-1, 1, leftMotorOutput);
		rightMotorOutput = coerce(-1, 1, rightMotorOutput);
		/*
		 * System.out.printf("l:%d r:%d\n",
		 * frontRight.getSensorCollection().getQuadraturePosition(),
		 * -backLeft.getSensorCollection().getQuadraturePosition());
		 */
//		lastMoveValue = moveValue;
		setRaw(leftMotorOutput, rightMotorOutput);
	}

	private static double coerce(double min, double max, double value) {
	        return Math.max(min, Math.min(value, max));
	    }
	public void setRaw(double left, double right) {
		backLeft.set(left);
		frontRight.set(-right);
		safetyHelper.feed();
	}

	@Override
	public void setExpiration(double timeout) {
		safetyHelper.setExpiration(timeout);
		
	}

	@Override
	public double getExpiration() {
		// TODO Auto-generated method stub
		return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
		frontLeft.stopMotor();
		frontRight.stopMotor();
		backLeft.stopMotor();
		backRight.stopMotor();
		safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		// TODO Auto-generated method stub
		safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		// TODO Auto-generated method stub
		return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Robot Drive";
	}

}

