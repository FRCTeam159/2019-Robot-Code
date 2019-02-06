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
import frc.robot.RobotMap;
import frc.robot.commands.GrabberCommands;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {
  private Servo armMover = new Servo(RobotMap.ARM_SERVO);
  private TalonSRX grabberMotor = new TalonSRX(RobotMap.GRABBER_MOTOR);;
  private DoubleSolenoid grabPneumatic = new DoubleSolenoid(RobotMap.GRABBER_PISTON_ID, RobotMap.PISTON_FORWARD,
      RobotMap.PISTON_REVERSE);;
  private double ejectValue = 0.5;
  private double grabValue = -0.5;
    

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new GrabberCommands());
  }
  public void closeClaw(){
    grabPneumatic.set(DoubleSolenoid.Value.kReverse);
  }
  public void openClaw(){
   grabPneumatic.set(DoubleSolenoid.Value.kForward);
  }
  public void eject(){
    grabberMotor.set(ControlMode.PercentOutput, ejectValue);
  }
  public void grab(){
    grabberMotor.set(ControlMode.PercentOutput, grabValue);
  }
  public void hold(){
    grabberMotor.set(ControlMode.PercentOutput, 0);

  }
  public void dropGrabber(){
    armMover.set(RobotMap.GRABBER_SERVO_VALUE);
  }
}