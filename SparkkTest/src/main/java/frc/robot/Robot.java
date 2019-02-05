/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANDigitalInput;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 * 
 * In order to use the Spark Max Motor, set the following settings in SPark MAx Client:
 * ~CAN ID = 8
 * ~Idle Mode = Brake
 * ~Deadband = 0 (It doesn't work.) 
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 8;
  private static final int kJoystickPort = 0;

  private CANSparkMax m_motor;
  private Joystick m_joystick;
  private CANEncoder m_encoder;
  private CANDigitalInput m_forward_limit;
  private CANDigitalInput m_backward_limit;
  private double motorvalue = 0;
  private double motorsteps = 300;

  @Override
  public void robotInit() {
    m_motor = new CANSparkMax(kMotorPort,CANSparkMaxLowLevel.MotorType.kBrushless);
    m_joystick = new Joystick(kJoystickPort);
    m_encoder = m_motor.getEncoder();
    m_forward_limit = m_motor.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
    m_backward_limit = m_motor.getReverseLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
    
  }

  public void teleopInit() {
    System.out.println("Default teleopInit() method.");
    m_forward_limit.enableLimitSwitch(true);
    m_backward_limit.enableLimitSwitch(true);
    motorvalue = 0;
    //m_forward_limit.enableLimitSwitch​(true);
  }

  @Override
  public void teleopPeriodic() {
   // System.out.println((m_joystick.getRawAxis(1)));
   /*if (motorvalue > 1.0){
    m_motor.set (0);
    return;
   } */
   double position = m_encoder.getPosition();
   double velocity = m_encoder.getVelocity();
   System.out.println(position + "," + velocity);
   //m_motor.set (motorvalue);
double stick = -m_joystick.getRawAxis(1);
   //motorvalue += 1.0 / motorsteps;
   if(Math.abs(stick) < .2){
   stick = 0;
   }
   m_motor.set(stick);
  }

}
