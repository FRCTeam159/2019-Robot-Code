package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.RobotMap;
import frc.robot.commands.ElevatorCommands;

/**
 *
 */
public class Elevator extends Subsystem implements PIDSource, PIDOutput, RobotMap{
    private TalonSRX elevatorMotor;
   private DoubleSolenoid elevatorPneumatic = new DoubleSolenoid(ELEVATOR_PISTON_FORWARD,
            ELEVATOR_PISTON_REVERSE);
            
    /*
     * Known good PID settings (use as fallbacks)
     * 
     * P = 0.125 I = 0 D = 0.75 F = 0
     */
    // all distance units are in inches
    private static final double P = 0.15;
    private static final double I = 0.0;
    private static final double D = 0.75;
    private static final double F = 0.0;

    private static final double WHEEL_DIAMETER = 1.75;
    private static final int ENCODER_EDGES = 4;
    private static final int ENCODER_TICKS = 1024;
    private static final double GEAR_RATIO = 1;
    private static final double TICKS_PER_REVOLUTION = GEAR_RATIO * ENCODER_TICKS * ENCODER_EDGES;
    private static final double INCHES_PER_REV = Math.PI * WHEEL_DIAMETER;
    private static final double TICKS_PER_INCH = TICKS_PER_REVOLUTION / INCHES_PER_REV;

    private static final double MIN_HEIGHT = 0;

    public static final double START_HEIGHT = 4;
    public static final double CARGO_HATCH_HEIGHT = 16;
    public static final double CARGO_BALL_HEIGHT = 36;
    public static final double ROCKET_BALL_HEIGHT_LOW = 27.5;
    public static final double DELTA_TARGET_HEIGHT = 28;
    public static final double ROCKET_TOP_BALL_HEIGHT = 2 * (DELTA_TARGET_HEIGHT) + ROCKET_BALL_HEIGHT_LOW;
    public static final double MAX_HEIGHT = ROCKET_TOP_BALL_HEIGHT;

    public static final double MAX_SPEED = 60;
    public static final double CYCLE_TIME = 0.02;
    public static final double MOVE_RATE = CYCLE_TIME * MAX_SPEED;
    public static final double BASE_DISTANCE_TO_GROUND = 6;

    private PIDController pidController;
    private PIDSourceType pidType = PIDSourceType.kDisplacement;

    private double elevatorTarget = 0;

    public void initDefaultCommand() {
        setDefaultCommand(new ElevatorCommands());
    }

    public Elevator() {
        super();
        elevatorMotor = new TalonSRX(RobotMap.ELEVATOR_MOTOR);
        double elevatorCurrent = elevatorMotor.getOutputCurrent();
        System.out.println(elevatorCurrent);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, RobotMap.TIMEOUT);
        elevatorMotor.setStatusFramePeriod(com.ctre.phoenix.motorcontrol.StatusFrameEnhanced.Status_3_Quadrature,
                RobotMap.ENCODER_STATUS_FRAME_PERIOD, RobotMap.TIMEOUT);
        elevatorMotor.set(ControlMode.PercentOutput, 0);
        // Current limits set
        pidController = new PIDController(P, I, D, F, this, this, 0.01);
        pidController.setOutputRange(-1.0, 1.0);
        reset();
         //Code for Hard limit switches for talons
        elevatorMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 10);
        elevatorMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 10);
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    public double getCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    public double getPosition() {
        double pos = (elevatorMotor.getSensorCollection().getQuadraturePosition() / TICKS_PER_INCH) * 4;
        SmartDashboard.putNumber("Elevator", pos);
        // SmartDashboard.putNumber("ElevatorCurrent",
        // elevatorMotor.getOutputCurrent());
        return pos;
    }

    public void setElevatorTarget(double value) {
        value = value < MIN_HEIGHT ? MIN_HEIGHT : value;
        value = value > MAX_HEIGHT ? MAX_HEIGHT : value;
        elevatorTarget = value - BASE_DISTANCE_TO_GROUND;
        pidController.setSetpoint(elevatorTarget);
    }

    public double getElevatorTarget() {
        return elevatorTarget + BASE_DISTANCE_TO_GROUND;
    }

    public void set(double value) {
        elevatorMotor.set(ControlMode.PercentOutput, value);
    }

    public void reset() {
        elevatorTarget = 0;
        pidController.reset();
        pidController.enable();
        pidController.setSetpoint(elevatorTarget);
        elevatorMotor.set(ControlMode.PercentOutput, 0);
        elevatorMotor.getSensorCollection().setQuadraturePosition(0, RobotMap.ENCODER_TIMEOUT);
    }

    public boolean isAtZero() {
        // TODO: implement lower limit switch
        return elevatorMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean isAtTop() {
        return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public void enable() {
        pidController.enable();
    }

    public void disable() {
        pidController.disable();
    }

    private double metersToInches(double meters) {
        return (100 * meters) / 2.54;
    }

    public void tiltElevator(boolean forward) {
        if (forward)
            elevatorPneumatic.set(DoubleSolenoid.Value.kForward);
        else
            elevatorPneumatic.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void pidWrite(double value) {
        elevatorMotor.set(ControlMode.PercentOutput, value);
    }

    @Override
    public double pidGet() {
        return getPosition();
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSourceType) {
        pidType = pidSourceType;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidType;
    }

}