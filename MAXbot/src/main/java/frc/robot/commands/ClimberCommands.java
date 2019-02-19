/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ClimberCommands extends Command implements RobotMap {
  public ClimberCommands() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("ClimberCommands initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Joystick stick = OI.stick;
    boolean isClimberButtonPressed = stick.getRawButton(CLIMB_BUTTON);
    if (isClimberButtonPressed) {
      Robot.climber.setClimbValue();
    } else {
      Robot.climber.setZeroValue();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("ClimberCommands end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("ClimberCommands interrupted");
  }
}
