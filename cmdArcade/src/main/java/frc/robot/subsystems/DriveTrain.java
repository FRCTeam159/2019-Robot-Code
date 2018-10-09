/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithGamepad;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveTrain extends Subsystem implements MotorSafety, RobotMap{

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
	private MotorSafetyHelper safetyHelper = new MotorSafetyHelper(this);
	 
	
	private WPI_TalonSRX frontLeft;
	private WPI_TalonSRX frontRight;
	private WPI_TalonSRX backLeft;
	private WPI_TalonSRX backRight;
	

  public DriveTrain(){
    frontLeft = new WPI_TalonSRX(FRONT_LEFT);
		frontRight = new WPI_TalonSRX(FRONT_RIGHT);
		backLeft = new WPI_TalonSRX(BACK_LEFT);
		backRight = new WPI_TalonSRX(BACK_RIGHT);
  }
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new DriveWithGamepad());
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
		return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		frontLeft.stopMotor();
		frontRight.stopMotor();
		backLeft.stopMotor();
		backRight.stopMotor();
		safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Robot Drive";
	}
}
