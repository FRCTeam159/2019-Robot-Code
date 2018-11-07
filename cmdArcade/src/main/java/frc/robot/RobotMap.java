/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {
  // Motor IDs
  public static final int FRONT_LEFT = 3;
  public static final int BACK_LEFT = 4;

  public static final int FRONT_RIGHT = 2;
  public static final int BACK_RIGHT = 1;

  // Controller IDs
  public static final int DRIVER = 0;
  // These are button IDs
  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;
  // General Constants
  public static final int TIMEOUT = 10;
  public static final int ENCODER_TIMEOUT = 10;
  public static final int ENCODER_WINDOW_SIZE = 4;
  public static final int ENCODER_STATUS_FRAME_PERIOD = 4;

  public static final int LEFT_POSITION = 0;
  public static final int CENTER_POSITION = 1;
  public static final int RIGHT_POSITION = 2;
  public static final int ILLEGAL_POSITION = 3;
}
