/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Autonomous extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Autonomous() {
    addSequential(new DriveToTarget(48.0));
    //addSequential(new DriveStraight(5));
    //addSequential(new DrivePath(5, 3));
    //addSequential(new DrivePath(5, 0));
  }
}
