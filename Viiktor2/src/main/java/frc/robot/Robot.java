/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionProcess;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap {
  public static DriveTrain m_drivetrain = new DriveTrain();
  public static OI m_oi;
//  public static Cameras m_cameras = new Cameras();

  Command m_autonomousCommand;
  SendableChooser<Integer> positionChooser = new SendableChooser<>();
  public static boolean useGyro = false;
  public static boolean calibrate = false;
  public static int robotPosition = CENTER_POSITION;
  public static boolean publishPath = false;
  public static double P=0.02;
  public static double I=1.0E-4;
  public static double D=0;
  public static boolean isAuto = false;
  public static boolean isTele = false;
  public static boolean doAuto = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  VisionProcess vision;

  @Override
  public void robotInit() {
    m_oi = new OI();
    vision = new VisionProcess();
    //vision.init();
    //vision.start();
    putValuesOnSmartDashboard();
    // m_chooser.addDefault("Default Auto", new ExampleCommand());
    // chooser.addObject("My Auto", new MyAutoCommand());
    // SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    System.out.println("Disabled");
    isTele = false;
    isAuto = false;
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    getDashboardData();
    if(doAuto){
    isAuto = true;
    isTele = false;
    }
    else{
      isAuto = false;
      isTele = true;
    }
    System.out.println("autonomous init");
    m_drivetrain.reset();

    robotPosition = getPosition();
    if(doAuto){
    CommandGroup autonomousCommand = new Autonomous();
    autonomousCommand.start();
  } 

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    isAuto = false;
    isTele = true;
    m_drivetrain.reset();
    System.out.println("Teleop Init");
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private int getPosition() {
    return positionChooser.getSelected();
  }

  private void putValuesOnSmartDashboard() {
    positionChooser.addObject("Left", 0);
    positionChooser.addDefault("Center", 1);
    positionChooser.addObject("Right", 2);
    SmartDashboard.putData("Position", positionChooser);
    SmartDashboard.putNumber("MAX_VEL", PhysicalConstants.MAX_VEL);
    SmartDashboard.putNumber("MAX_ACC", PhysicalConstants.MAX_ACC);
    SmartDashboard.putNumber("MAX_JRK", PhysicalConstants.MAX_JRK);
    SmartDashboard.putNumber("KP", PhysicalConstants.KP);
    SmartDashboard.putNumber("GFACT", PhysicalConstants.GFACT);
    SmartDashboard.putBoolean("Use Gyro", useGyro);
    SmartDashboard.putString("Target", "Calculating");
    SmartDashboard.putBoolean("Calibrate", calibrate);
    SmartDashboard.putBoolean("Publish Path", publishPath);
    SmartDashboard.putNumber("P", P);
    SmartDashboard.putNumber("I", I);
    SmartDashboard.putNumber("D", D);
    SmartDashboard.putBoolean("Do Auto", doAuto);
    }

  void getDashboardData() {
    useGyro = SmartDashboard.getBoolean("Use Gyro", useGyro);
    PhysicalConstants.MAX_VEL = SmartDashboard.getNumber("MAX_VEL", PhysicalConstants.MAX_VEL);
    PhysicalConstants.MAX_ACC = SmartDashboard.getNumber("MAX_ACC", PhysicalConstants.MAX_ACC);
    PhysicalConstants.MAX_JRK = SmartDashboard.getNumber("MAX_JRK", PhysicalConstants.MAX_JRK);
    PhysicalConstants.GFACT = SmartDashboard.getNumber("GFACT", PhysicalConstants.GFACT);
    PhysicalConstants.KP = SmartDashboard.getNumber("KP", PhysicalConstants.KP);
    calibrate = SmartDashboard.getBoolean("Calibrate", calibrate);
    publishPath = SmartDashboard.getBoolean("Publish Path", publishPath);
    P=SmartDashboard.getNumber("P", P);
    I=SmartDashboard.getNumber("I", I);
    D=SmartDashboard.getNumber("D", D);
    doAuto = SmartDashboard.getBoolean("Do Auto", false);

  }
}

