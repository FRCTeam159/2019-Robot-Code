/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.ClimberCommands;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxFrames;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem implements RobotMap {
  private CANSparkMax climberMotor;
  private double climbValue = 0.2;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Climber(){
    climberMotor = new CANSparkMax(CLIMBER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ClimberCommands());
  }

  void set(double value){
    climberMotor.set(value);
  }

  public void setClimbValue(){
    climberMotor.set(climbValue);
  }
  public void setZeroValue(){
    climberMotor.set(0);
  }

}