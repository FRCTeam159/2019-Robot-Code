/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.GrabberCommands;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem implements RobotMap {
  private Servo armMover = new Servo(RobotMap.ARM_SERVO);
  private TalonSRX grabberMotor = new TalonSRX(RobotMap.GRABBER_MOTOR);;
  private DoubleSolenoid grabPneumatic = new DoubleSolenoid(GRABBER_PISTON_FORWARD, GRABBER_PISTON_REVERSE);
  private double ejectValue = 0.5;
  private double grabValue = -0.5;
  private boolean clawOpen = true;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GrabberCommands());
  }

  public boolean isClawOpen() {
    return clawOpen;
  }

  public void closeClaw() {
    grabPneumatic.set(DoubleSolenoid.Value.kReverse);
    clawOpen = false;
    SmartDashboard.putBoolean("ClawOpen", clawOpen);
  }

  public void openClaw() {
    grabPneumatic.set(DoubleSolenoid.Value.kForward);
    clawOpen = true;
    SmartDashboard.putBoolean("ClawOpen", clawOpen);
  }

  public void eject() {
    if (Robot.cargoMode) {
      grabberMotor.set(ControlMode.PercentOutput, ejectValue);
    } else {
      grabberMotor.set(ControlMode.PercentOutput, grabValue);
    }
  }

  public void grab() {
    if (Robot.cargoMode) {
      grabberMotor.set(ControlMode.PercentOutput, grabValue);
    } else {
      grabberMotor.set(ControlMode.PercentOutput, ejectValue);
    }
  }

  public void hold() {
    grabberMotor.set(ControlMode.PercentOutput, 0);

  }

  public void dropGrabber() {

    armMover.set(0);

  }

  public void disableTilting() {
    armMover.setDisabled();

  }
}
