/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Button;
import edu.wpi.first.wpilibj.Timer;

public class GrabberCommands extends Command implements RobotMap {
  Timer timer = new Timer();
  Button toggleGrabber = new Button(ARMS_TOGGLE_BUTTON);
  Button tiltForwardButton = new Button(TILT_FORWARD_BUTTON);
  Button tiltBackButton = new Button(TILT_BACK_BUTTON);
  static final int NOT_TILTING = 0;
  static final int TILTING_UP = 1;
  static final int TILTING_DOWN = 2;
  int tilting = NOT_TILTING;
  static final double tiltTime = 0.1; // seconds

  public GrabberCommands() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.grabber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!(Robot.isTele))
      return;
    Joystick stick = OI.stick;
    boolean intakeButtonPressed = stick.getRawButton(INTAKE_BUTTON);
    boolean outputButtonPressed = stick.getRawButton(OUTPUT_BUTTON);
    if (intakeButtonPressed)
      Robot.grabber.grab();
    else if (outputButtonPressed)
      Robot.grabber.eject();
    else
      Robot.grabber.hold();
    if (toggleGrabber.isPressed()) {
      if (Robot.grabber.isClawOpen())
        Robot.grabber.closeClaw();
      else
        Robot.grabber.openClaw();
    }
    if (tiltForwardButton.isPressed()) {
      tilting = TILTING_UP;
      timer.reset();
    } else if (tiltBackButton.isPressed()) {
      tilting = TILTING_DOWN;
      timer.reset();
    }
    manageTilt();
  }

  void manageTilt() {
    switch (tilting) {
    case NOT_TILTING:
      Robot.grabber.disableTilting();
      break;
    case TILTING_UP:
      if (timer.get() < tiltTime)
        Robot.grabber.dropGrabber(true);
      else
        tilting = NOT_TILTING;
      break;
    case TILTING_DOWN:
      if (timer.get() < tiltTime)
        Robot.grabber.dropGrabber(false);
      else
        tilting = NOT_TILTING;
      break;
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
