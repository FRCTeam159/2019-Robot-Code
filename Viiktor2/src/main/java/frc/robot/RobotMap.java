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
  public static final int ELEVATOR_MOTOR = 5;

  // Controller IDs
  public static final int DRIVER = 0;
  // These are button IDs
  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;
  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;
  public static final int LEFT_BUMPER_BUTTON = 5;
  public static final int RIGHT_BUMPER_BUTTON = 6;
  public static final int RESET_ELEVATOR_BUTTON = 8;
  public static final int INTAKE_BUTTON = 1;
  public static final int OUTPUT_BUTTON = 3;
  public static final int ARMS_TOGGLE_BUTTON = 4;
  public static final int GEAR_BUTTON = 7;
  public static final int CLIMB_BUTTON = 2;
  public static final int TILT_GRABBER_BUTTON = 9;
  public static final int TILT_ELEVATOR_BUTTON = 10;
  // General Constants
  public static final int TIMEOUT = 10;
  public static final int ENCODER_TIMEOUT = 10;
  public static final int ENCODER_WINDOW_SIZE = 4;
  public static final int ENCODER_STATUS_FRAME_PERIOD = 4;

  public static final int LEFT_POSITION = 0;
  public static final int CENTER_POSITION = 1;
  public static final int RIGHT_POSITION = 2;
  public static final int ILLEGAL_POSITION = 3;
  // POV Buttons
  public static final int HATCH_MODE = 90;
  public static final int CARGO_MODE = 270;
  public static final int ELEVATOR_STOP = 180;
  public static final int ELEVATOR_RESUME = 0;
}
