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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.testCommand;

/**
 * Add your docs here.
 */
public class Talontest extends Subsystem implements RobotMap {
  private TalonSRX motor;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Talontest() {
    motor = new TalonSRX(TESTMOTOR);

    /*motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    motor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, 2,  10);
    motor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, 2,  10);
    */  
//  motor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, TESTMOTOR,  10);
   // motor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, TESTMOTOR,  10);

    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed,  10);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed,  10);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new testCommand());
  }

  public boolean isAtZero() {
    return motor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public boolean isAtTop() {
    return motor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public void set(double d) {
    motor.set(ControlMode.PercentOutput, d);
  }
  public double getPosition(){
  return 0;
  }
}
