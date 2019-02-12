package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorCommands extends Command implements RobotMap{

    private boolean goingToBottom = false;
    private boolean goingToTop = false;
    private boolean goingToSwitch = false;
    private final int UNINITIALIZED = 0;
    private final int ELEVATOR_UPRIGHT = 1;
    private final int ELEVATOR_AT_ZERO = 2;
    private final int ELEVATOR_INITIALIZED = 3;
    private final int ELEVATOR_AT_TARGET = 4;
    private int state = UNINITIALIZED;
    private double setPoint = 0;
    edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

    public ElevatorCommands() {
        // Use requires() here to declare subsystem dependencies eg. requires(chassis);
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        // Robot.elevator.setElevatorTarget(0);
        Robot.elevator.enable();
        printInitializeMessage();
        state = UNINITIALIZED;
        timer.start();
        timer.reset();
        Robot.elevator.tiltElevator();

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        switch (state) {
        case UNINITIALIZED:
            double tm = timer.get();
            if (tm > 1.0) {
                state = ELEVATOR_UPRIGHT;
            }
            break;
        case ELEVATOR_UPRIGHT:
            goToZeroLimit();
            break;
        case ELEVATOR_AT_ZERO:
            goToHatchHeight();
            state = ELEVATOR_INITIALIZED;
            break;
        case ELEVATOR_INITIALIZED:
            manualOperate();
            break;
        }
    }

    void manualOperate(){
        Joystick stick = OI.stick;

        boolean hatchButtonPressed = stick.getRawButton(RESET_ELEVATOR_BUTTON);
        boolean rightBumperPressed = stick.getRawButton(RIGHT_BUMPER_BUTTON);
        boolean leftBumperPressed = stick.getRawButton(LEFT_BUMPER_BUTTON);
        double rightTriggerPressed = stick.getRawAxis(RIGHT_TRIGGER);
        double leftTriggerPressed = stick.getRawAxis(LEFT_TRIGGER);
        if (hatchButtonPressed)
            setPoint = Elevator.CARGO_HATCH_HEIGHT;
        else if (rightBumperPressed)
            setPoint += Elevator.DELTA_TARGET_HEIGHT;
        else if (leftBumperPressed)
            setPoint -= Elevator.DELTA_TARGET_HEIGHT;
        else if (rightTriggerPressed > 0)
            setPoint += Elevator.MOVE_RATE;
        else if (leftTriggerPressed > 0)
            setPoint -= Elevator.MOVE_RATE;
        Robot.elevator.setElevatorTarget(setPoint);
        setPoint = Robot.elevator.getElevatorTarget();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        // Robot.elevator.reset();
        printEndMessage();
    }

    // Called when another command which requires one or more of the same subsystems
    // is scheduled to run
    protected void interrupted() {
        printInterruptedMessage();
        end();
    }

    private void printInitializeMessage() {
        System.out.println("Elevator.initialize");
    }

    private void printEndMessage() {
        System.out.println("Elevator.end");
    }

    private void printInterruptedMessage() {
        System.out.println("Elevator.interrupted");
    }

    public void goToZeroLimit() {
        Robot.elevator.set(-0.1);
        if (Robot.elevator.isAtZero()) {
            state = ELEVATOR_AT_ZERO;
            setPoint = 0;
        }
    }

    public void goToHatchHeight() {
        Robot.elevator.setElevatorTarget(Elevator.CARGO_HATCH_HEIGHT);
        setPoint = Robot.elevator.getElevatorTarget();
    }
}
