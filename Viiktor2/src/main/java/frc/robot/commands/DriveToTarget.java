/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.GripPipeline;
import frc.robot.subsystems.VisionProcess;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToTarget extends Command implements PIDSource, PIDOutput {
  NetworkTable table;
  double targetDistance = 12.0;
  private static final double kP = 0.02;
  private static final double minError = kP * 3;
  private double error = 0;
  private PIDController pid;

  private static final double P = 0.02;
	private static final double I = 0.001;
	private static final double D = 0.0;
	private static final double TOL = 0.05;

  public DriveToTarget(double d) {
    pid = new PIDController(P, I, D, this, this);
    targetDistance = d;
  }

 
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("TargetData");
    System.out.println("DriveToTarget.initialize");
    SmartDashboard.putNumber("RangeError", 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double range = table.getEntry("Range").getDouble(0.0);
    error = kP * (range - targetDistance); 
    if (error < 0) {
      error = 0;
    }
    SmartDashboard.putNumber("RangeError", error / kP);
    Robot.m_drivetrain.arcadeDrive(error, 0);
    
    //System.out.println("Range = " + range);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    return error < minError? true : false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("DriveToTarget.end");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  @Override
  public void pidWrite(double output) {

  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {

  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return  PIDSourceType.kDisplacement;
  }

  @Override
  public double pidGet() {
    return 0;
}
}
