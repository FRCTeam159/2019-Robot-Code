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

public class GrabberCommands extends Command implements RobotMap{
  edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  static final int UNINITIALIZED = 0;
  static final int ARMS_OPEN = 1;
  static final int GRABBER_UPRIGHT = 2;
  int state = UNINITIALIZED;

  public GrabberCommands() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.grabber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.start();
    timer.reset();
    Robot.grabber.openClaw();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    switch (state) {
    case UNINITIALIZED:
      if (timer.get() > 0.5){
        state = ARMS_OPEN;
        Robot.grabber.dropGrabber();
        timer.reset();
      }
      break;
    case ARMS_OPEN:
      if (timer.get() > 0.75)
        state = GRABBER_UPRIGHT;
      else
        Robot.grabber.dropGrabber();
      break;
    case GRABBER_UPRIGHT:
      manualOperate();
      break;
    }
  }

  void manualOperate() {
    Joystick stick = OI.stick;

    boolean intakeButtonPressed = stick.getRawButton(INTAKE_BUTTON);
    boolean outputButtonPressed = stick.getRawButton(OUTPUT_BUTTON);
    boolean toggleButtonPressed = stick.getRawButton(ARMS_TOGGLE_BUTTON);
    if (intakeButtonPressed)
      Robot.grabber.grab();
    else if (outputButtonPressed)
      Robot.grabber.eject();
    else 
      Robot.grabber.hold();
    if (toggleButtonPressed){
      if (Robot.grabber.isClawOpen())
        Robot.grabber.closeClaw();
      else 
        Robot.grabber.openClaw();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

}
