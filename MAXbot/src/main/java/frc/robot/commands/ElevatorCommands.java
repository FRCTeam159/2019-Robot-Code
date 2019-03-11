package frc.robot.commands;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Button;

/**
 *
 */
public class ElevatorCommands extends Command implements RobotMap {

    private double setPoint = 0;

    Button upDeltaButton = new Button(RIGHT_BUMPER_BUTTON);
    Button downDeltaButton = new Button(LEFT_BUMPER_BUTTON);
    Button resetButton = new Button(RESET_ELEVATOR_BUTTON);
    Button tiltButton = new Button(TILT_ELEVATOR_BUTTON);


    public ElevatorCommands() {
        // Use requires() here to declare subsystem dependencies eg. requires(chassis);
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        System.out.println("ElevatorCommands initialized");
        Robot.elevator.enable();
        // Robot.elevator.setElevatorTarget(0);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Joystick stick = OI.stick;
        int direction = OI.stick.getPOV(0);
        // if(!(Robot.isTele))
        // return;
        if(direction == 90){
            Robot.hatchMode = true;
            Robot.cargoMode = false;
        } else if (direction == 270){
            Robot.hatchMode = false;
            Robot.cargoMode = true;
        }
        double rightTriggerPressed = stick.getRawAxis(RIGHT_TRIGGER);
        double leftTriggerPressed = stick.getRawAxis(LEFT_TRIGGER);
       if (resetButton.isPressed()){
           if(Robot.hatchMode){
           setPoint = Elevator.BASE_HATCH_HEIGHT;
           } else{
            setPoint = Elevator.MIN_HEIGHT;
           }
       }
        else if (upDeltaButton.isPressed())
           setPoint += Elevator.DELTA_TARGET_HEIGHT;
        else if (downDeltaButton.isPressed())
            setPoint -= Elevator.DELTA_TARGET_HEIGHT;
         if (rightTriggerPressed > 0)
            setPoint += Elevator.MOVE_RATE;
        else if (leftTriggerPressed > 0)
            setPoint -= Elevator.MOVE_RATE;
        else if (tiltButton.isPressed()) {
            if (Robot.elevator.isTilted()) {
                Robot.elevator.tiltElevator(true);
            } else {
                Robot.elevator.tiltElevator(false);
            }
        }

        Robot.elevator.setElevatorTarget(setPoint);
        setPoint = Robot.elevator.getElevatorTarget();

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        System.out.println("ElevatorCommands end");
        // Robot.elevator.reset();
    }

    // Called when another command which requires one or more of the same subsystems
    // is scheduled to run
    protected void interrupted() {
        System.out.println("ElevatorCommands interrupted");
        end();
    }

}
