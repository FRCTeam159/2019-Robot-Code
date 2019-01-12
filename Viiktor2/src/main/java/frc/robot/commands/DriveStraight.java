package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command implements PIDSource, PIDOutput {
	private PIDController pid;
	private PIDSourceType type = PIDSourceType.kDisplacement;
	private final double distance;
	private static final double tolerance = 0.1;
	private static final boolean lastOntarget = false;
	private boolean started = false;
	private static final boolean debug = true;
	private static final double P = 0.25;
	private static final double I = 0.002;
	private static final double D = 0.0;
	private static final double TOL = 0.05;
	private static final double intime = 0.025;
	private Timer timer = new Timer();

	public DriveStraight(double distance) {
		this.distance = distance;
		pid = new PIDController(P, I, D, this, this);
		pid.setAbsoluteTolerance(TOL);
		requires(Robot.m_drivetrain);
	}

//	 Called just before this Command runs the first time
	protected void initialize() {
		System.out.printf("DriveStraight.initialize distance = %f\n", distance);
		pid.reset();
		pid.setSetpoint(distance);
		pid.disable();
		Robot.m_drivetrain.enable();
		Robot.m_drivetrain.reset();
		started = false;
		timer.start();

	}

//	 Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (!started && timer.get() > intime && !pid.isEnabled()) {
			pid.reset();
			pid.enable();
			started = true;
		}
	}

//	 Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (timer.get() > 3.0) {
			printFinishedMessage();
			return true;
		}
		return pid.onTarget();
	}

//	 Called once after isFinished returns true
	protected void end() {
		Robot.m_drivetrain.disable();
		pid.disable();
		started = false;
		
		printEndMessage();
	}

//	 Called when another command which requires one or more of the same subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		type = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return type;
	}

	@Override
	public double pidGet() {
		if (!started) {
			return 0;
		}
		if (debug) {
			double time = timer.get();
			System.out.printf("time=%f leftDistance=%g rightDistance=%g distance=%g\n", time * 1000, Robot.m_drivetrain.getLeftDistance(), Robot.m_drivetrain.getRightDistance(), Robot.m_drivetrain.getDistance());
		}
		return Robot.m_drivetrain.getDistance();
	}

	@Override
	public void pidWrite(double output) {
		if (started) {
			Robot.m_drivetrain.setRaw(output, output);
		}
	}
	
	private void printEndMessage() {
		System.out.println("drivestraight.end");
	}
	
	private void printFinishedMessage() {
		System.out.println("timer expired");
	}
}
