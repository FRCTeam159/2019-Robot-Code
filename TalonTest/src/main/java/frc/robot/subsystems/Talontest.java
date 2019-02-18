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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.testCommand;

/**
 * Add your docs here.
 */
public class Talontest extends Subsystem implements RobotMap {
  private TalonSRX motor;
  private static final double WHEEL_DIAMETER = 1.75;
  private static final int ENCODER_EDGES = 4;
  private static final int ENCODER_TICKS = 1024;
  private static final double GEAR_RATIO = 1;
  private static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_TICKS * ENCODER_EDGES;
  private static final double INCHES_PER_REV = Math.PI * WHEEL_DIAMETER;
  private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REV;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Talontest() {
    motor = new TalonSRX(ARM_SERVO);

    /*
     * motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
     * motor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
     * LimitSwitchNormal.NormallyOpen, 2, 10);
     * motor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
     * LimitSwitchNormal.NormallyOpen, 2, 10);
     */
    // motor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
    // LimitSwitchNormal.NormallyClosed, TESTMOTOR, 10);
    // motor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX,
    // LimitSwitchNormal.NormallyClosed, TESTMOTOR, 10);
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    motor.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature, 4, 10);
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
    reset();
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
  public void reset(){
    motor.set(ControlMode.PercentOutput, 0);
        motor.getSensorCollection().setQuadraturePosition(0, 10);
        log();
  }
  public void set(double d) {
    motor.set(ControlMode.PercentOutput, d);
  }

  public double getPosition() {
    double pos = (motor.getSensorCollection().getQuadraturePosition() / TICKS_PER_INCH) * 4;
    
    return pos;
  }
  public void log(){
    SmartDashboard.putNumber("Elevator", getPosition());
    SmartDashboard.putBoolean("lowerlimit", isAtZero());
    SmartDashboard.putBoolean("upperlimit", isAtTop());
  }
}
