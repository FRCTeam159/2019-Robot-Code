/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* TODO
Elevator
  Test Quad encoder on elevator
  Test limit switches
  Calibrate Elevator position based on input
  Redefine setpoints 
  Control Elevator tilt
  Check elevator commands for relevancy
  remap buttons
Grabber
  open and close grabber
  Find out grabber functions
  eject cargo
  place hatches
*/
package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.VisionProcess;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.LightSystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap {
  public static DriveTrain drivetrain = new DriveTrain();
  public static OI m_oi;
  public static Elevator elevator = new Elevator();
  public static Grabber grabber = new Grabber();
  public static Climber climber = null;
  public static final double MAX_ELEVATOR_CURRENT = 40;
  Compressor compressor1 = new Compressor();
  public static LightSystem lightSystem = new LightSystem();
  // public static Cameras m_cameras = new Cameras();

  CommandGroup autonomousCommand;
  //SendableChooser<Integer> positionChooser = new SendableChooser<>();
  public static boolean useGyro = false;
  public static boolean calibrate = false;
  //public static int robotPosition = CENTER_POSITION;
  public static boolean publishPath = false;
  public static boolean isAuto = false;
  public static boolean isTele = false;
  public static boolean doAuto = false;
  public static boolean haveAuto = false;
  public static boolean haveClimber = false;
  static UsbCamera camera1;
  static UsbCamera camera2;
  public static int imageWidth = 320;
  public static int imageHeight = 240;
  static int FPS = 30;
  public static boolean hatchMode = true;
  public static boolean cargoMode = false;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  // VisionProcess vision;

  @Override
  public void robotInit() {
    if (haveClimber)
      climber = new Climber();
    m_oi = new OI();
    compressor1.start();
    camera1 = CameraServer.getInstance().startAutomaticCapture("Camera1", 0);
    camera1.setFPS(FPS);
    camera1.setResolution((int) imageWidth, (int) imageHeight);
    camera2 = CameraServer.getInstance().startAutomaticCapture("Camera2", 1);
    camera2.setFPS(FPS);
    camera2.setResolution((int) imageWidth, (int) imageHeight);

    autonomousCommand = new Autonomous();
    /*
     * vision = new VisionProcess(); vision.init(); vision.start();
     */
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
    if (doAuto) {
      isAuto = true;
      isTele = false;
    } else {
      isAuto = false;
      isTele = true;
    }

    getDashboardData();
    compressor1.start();
    //robotPosition = getPosition();

    // autonomousCommand.start();

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
    drivetrain.reset();
    compressor1.start();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
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

 // private int getPosition() {
   // return positionChooser.getSelected();
  //}

  private void putValuesOnSmartDashboard() {
    if (haveAuto) {
     // positionChooser.addObject("Left", 0);
      //positionChooser.addDefault("Center", 1);
      //positionChooser.addObject("Right", 2);
      //SmartDashboard.putData("Position", positionChooser);
      SmartDashboard.putNumber("MAX_VEL", PhysicalConstants.MAX_VEL);
      SmartDashboard.putNumber("MAX_ACC", PhysicalConstants.MAX_ACC);
      SmartDashboard.putNumber("MAX_JRK", PhysicalConstants.MAX_JRK);
      SmartDashboard.putNumber("KP", PhysicalConstants.KP);
      SmartDashboard.putNumber("GFACT", PhysicalConstants.GFACT);
      SmartDashboard.putBoolean("Use Gyro", useGyro);
      SmartDashboard.putString("Target", "Calculating");
      SmartDashboard.putBoolean("Calibrate", calibrate);
      SmartDashboard.putBoolean("Publish Path", publishPath);
    }
  }

  void getDashboardData() {
    if (haveAuto) {
      useGyro = SmartDashboard.getBoolean("Use Gyro", useGyro);
      PhysicalConstants.MAX_VEL = SmartDashboard.getNumber("MAX_VEL", PhysicalConstants.MAX_VEL);
      PhysicalConstants.MAX_ACC = SmartDashboard.getNumber("MAX_ACC", PhysicalConstants.MAX_ACC);
      PhysicalConstants.MAX_JRK = SmartDashboard.getNumber("MAX_JRK", PhysicalConstants.MAX_JRK);
      PhysicalConstants.GFACT = SmartDashboard.getNumber("GFACT", PhysicalConstants.GFACT);
      PhysicalConstants.KP = SmartDashboard.getNumber("KP", PhysicalConstants.KP);
      calibrate = SmartDashboard.getBoolean("Calibrate", calibrate);
      publishPath = SmartDashboard.getBoolean("Publish Path", publishPath);
    }
  }
  public static double round(double v){
    return 0.01*Math.round(v*100);
      }
}
