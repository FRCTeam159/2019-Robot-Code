/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveWithGamepad extends Command implements RobotMap{
  double powerScale = 2.0;
  double turnScale = 0.65;
  double moveExponent = 1;
  double turnExponent = 1;
  double xMinT = 0.18;
  double xMinO = 0;
  double zMinT = 0.035;
  double zMinO = 0;
  boolean useDeadband = true;
  public DriveWithGamepad() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick stick = OI.driverController;
    double zs=stick.getRawAxis(RobotMap.LEFT_JOYSTICK);
    double xs=stick.getRawAxis(RobotMap.RIGHT_JOYSTICK);
    double z = zs;
    double x = xs;
    if (useDeadband){
       z = quadDeadband(zMinT, zMinO, zs);
       x = quadDeadband(xMinT, xMinO, xs);
    }
    double moveAxis = -powerScale * z; // left stick - drive
    double turnAxis = powerScale * x; // right stick - rotate
    double moveValue = 0;
    double turnValue = 0;
    System.out.println("xs = " + xs + " x = " + x);
    System.out.println("zs = " + zs + " z = " + z);

    if (Math.abs(moveAxis) > 0.0) {
      moveValue = (moveAxis / Math.abs(moveAxis)) * Math.pow(Math.abs(moveAxis), moveExponent);
    }
    if (Math.abs(turnAxis) > 0.0) {
      turnValue = (turnAxis / Math.abs(turnAxis)) * Math.pow(Math.abs(turnAxis), turnExponent); // Math.abs the
                                                                                                // turnValue
    }

    turnValue *= Math.abs(moveValue) * (1 - turnScale) + turnScale;
    Robot.m_drivetrain.arcadeDrive(moveValue, turnValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  double quadDeadband(double minThreshold, double minOutput, double input)
  {
    if (input > minThreshold) {
      return ((((1 - minOutput) // 1 - minOutput/(1-minThreshold)^2 * (input-minThreshold)^2 + minOutput
          / ((1 - minThreshold)* (1 - minThreshold)))
          * ((input - minThreshold)* (input - minThreshold)))
          + minOutput);
    } else {
      if (input < (- minThreshold)) {
        return (((minOutput - 1) // minOutput - 1/(minThreshold - 1)^2 * (minThreshold + input)^2 - minOutput
            / ((minThreshold - 1)* (minThreshold - 1)))
            * ((minThreshold + input)* (minThreshold + input)))
            - minOutput;
      }
  
      else {
        return 0;
      }
    }
  }
}

