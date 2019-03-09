package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.PhysicalConstants;
import frc.robot.RobotMap;

import frc.robot.Robot;

//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DrivePath extends Command implements frc.robot.RobotMap {

    private static final double TIME_STEP = 0.02;
    private static final double wheelbase = inchesToMeters(26);
    private Trajectory trajectory;
    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
    private DistanceFollower leftFollower;
    private DistanceFollower rightFollower;
    private static final boolean printCalculatedTrajectory = false;
    private static final boolean printCalculatedPath = false;
    private static final boolean debugCommand = false;
    private static final boolean printPath = false;

    // private static int plotId = 0;
    private int pathIndex = 0;
    private int pathPoints = 0;

    private Timer timer = new Timer();
    private Timer pushTimer = new Timer();
    private boolean pushing = false;
    private final double runtime;

    double last_heading = 0;

    private static int plotCount = 0;
    public boolean useCornerSidePaths = true;

    private ArrayList<PathData> pathDataList = new ArrayList<>();
    // static NetworkTableInstance ti=NetworkTableInstance.getDefault();
    // static NetworkTable table=ti.getTable("datatable");
    // TODO use non deprecated class (edu.wpi.first.networktables.NetworkTable)
    private static NetworkTable table = NetworkTable.getTable("datatable");

    private double distance;
    private double offSet;

    // Using Jaci's conventions. 
    // Y values are inverted, left and right wheels are inverted, counter-clockwise is clockwise. Everything is mirrored.

    public DrivePath(double d, double y) {
        requires(Robot.drivetrain);

        distance = feetToMeters(d);
        offSet = feetToMeters(y);
        timer.start();
        timer.reset();

        timer.start();
        timer.reset();

       
        double KI = 0.0;
        double KD = 0.0;
        double KV = 1.0 / PhysicalConstants.MAX_VEL;
        double KA = 0.0;

        timer.start();
        timer.reset();

        trajectory = calculateTrajectory(distance, offSet, PhysicalConstants.MAX_VEL, PhysicalConstants.MAX_ACC, PhysicalConstants.MAX_JRK);
        
        if (trajectory == null) {
            SmartDashboard.putString("Target", "ERROR");
            runtime = 0;
            return;
        }

        pathPoints = trajectory.length();
        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);
        modifier.modify(wheelbase);
        runtime = trajectory.length() * TIME_STEP;

        // Generate the Left and Right trajectories using the original trajectory as the
        // center

        leftTrajectory = modifier.getLeftTrajectory(); // Get the Left Side
        rightTrajectory = modifier.getRightTrajectory(); // Get the Right Side

        leftFollower = new DistanceFollower(leftTrajectory);
        leftFollower.configurePIDVA(PhysicalConstants.KP, KI, KD, KV, KA);
        rightFollower = new DistanceFollower(rightTrajectory);
        rightFollower.configurePIDVA(PhysicalConstants.KP, KI, KD, KV, KA);
        System.out.format("trajectory length:%data runtime:%f calctime:%f\n", trajectory.length(), runtime,
                timer.get());

        if (printCalculatedTrajectory) {
            double time = 0;
            for (int i = 0; i < trajectory.length(); i++) {
                Segment segment = trajectory.get(i);
                System.out.format("%f %f %f %f %f %f \n", time, segment.x, segment.y, segment.velocity,
                        segment.acceleration, segment.heading);
                time += segment.dt;
            }
        }

        if (printCalculatedPath) {
            double time = 0;
            for (int i = 0; i < trajectory.length(); i++) {
                Segment centerSegment = trajectory.get(i);
                Segment leftSegment = leftTrajectory.get(i);
                Segment rightSegment = rightTrajectory.get(i);
                System.out.format("%f %f %f %f %f\n", time, leftSegment.x, leftSegment.y, rightSegment.x,
                        rightSegment.y);
                time += centerSegment.dt;
            }
        }
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (trajectory == null) {
            return;
        }

        printInitializeMessage();
        pathDataList.clear();
        if (!publishPathAllowed()) {
            plotCount = 0;
        }

        leftFollower.reset();
        rightFollower.reset();
        Robot.drivetrain.reset();
        timer.start();
        timer.reset();
        pathIndex = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (trajectory == null || pushing) {
            return;
        }
        double leftDistance = feetToMeters(Robot.drivetrain.getLeftDistance());
        double rightDistance = feetToMeters(Robot.drivetrain.getRightDistance());

        double leftPower = leftFollower.calculate(leftDistance);
        double rightPower = rightFollower.calculate(rightDistance);

        double turn = 0;

        double gh = -Robot.drivetrain.getHeading(); // Assuming the gyro is giving a value in degrees
        gh = unwrap(last_heading, gh);

        double th = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

        th = th > 180 ? th - 360 : th;
        double headingError = th - gh;
        if (Robot.useGyro) {
            turn = PhysicalConstants.GFACT * (-1.0 / 180.0) * headingError;
        }

        double lval = leftPower + turn;
        double rval = rightPower - turn;


        if (debugCommand) {
            System.out.format("%f %f %f %f %f %f %f\n", timer.get(), leftDistance, rightDistance, th, gh, rval, lval);
        }

        if (printPath) {
            debugPathError();
        }
        if (publishPathAllowed()) {
            int maxpoints = trajectory.length();
            if (pathIndex <= maxpoints)
                addPlotData();
        }
        pathIndex++;
        last_heading = gh;
        // this is reversed because we found it to be reversed, don't change unless you
        // know what you're doing
        Robot.drivetrain.setRaw(lval, rval);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (trajectory == null) {
            return true;
        }
        if ((leftFollower.isFinished() && rightFollower.isFinished()) ) {
        	Robot.drivetrain.disable();
            

        } else if (pushTimer.get() - runtime > 2) {
            System.out.println("DrivePath Timeout Expired");
            return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        printEndMessage();
        if (publishPathAllowed()) {
            publish(pathDataList, 6);
        }
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }

    double unwrap(double previous_angle, double new_angle) {
        double d = new_angle - previous_angle;
        d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
        return previous_angle + d;
    }

    private boolean publishPathAllowed() {
        return Robot.publishPath;
    }

    private Waypoint[] mirrorWaypoints(Waypoint[] waypoints) {
        Waypoint[] newWaypoints = new Waypoint[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            newWaypoints[i] = new Waypoint(waypoints[i].x, -waypoints[i].y, -waypoints[i].angle);
        }
        return newWaypoints;
    }

    private Waypoint[] waypointsInchesToMeters(Waypoint[] waypoints) {
        Waypoint[] newWaypoints = new Waypoint[waypoints.length];
        for (int i = 0; i < waypoints.length; i++) {
            newWaypoints[i] = new Waypoint(inchesToMeters(waypoints[i].x), inchesToMeters(waypoints[i].y),
                    waypoints[i].angle);
        }
        return newWaypoints;
    }

    private Trajectory calculateTrajectory(double d, double y, double maxVelocity,
            double maxAcceleration, double maxJerk) {
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
                Trajectory.Config.SAMPLES_FAST, TIME_STEP, maxVelocity, maxAcceleration, maxJerk);

        Waypoint[] waypoints = calculatePathWaypoints(d, y);
        if (waypoints == null) {
            return null;
        }
        Trajectory calculatedTrajectory = Pathfinder.generate(waypoints, config);
        /*
         * if (calculatedTrajectory == null) {
         * System.out.println("Uh-Oh! Trajectory could not be generated!\n"); }
         */
        if (waypoints != null) {
            for (Waypoint waypoint : waypoints) {
                System.out.println(waypoint.x + " " + waypoint.y + " " + waypoint.angle);
            }
        }
        return calculatedTrajectory;
    }

    private Waypoint[] calculatePathWaypoints(double d, double y) {
        Waypoint[] returnWaypoints = null;
        if (Robot.robotPosition == CENTER_POSITION)
            returnWaypoints = calculateStraightPoints(d);
        else 
            returnWaypoints = calculateHookpoints(d, y);
        return returnWaypoints;
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    private void debugPathError() {
        double leftDistance = 12 * (Robot.drivetrain.getLeftDistance());
        double rightDistance = 12 * (Robot.drivetrain.getRightDistance());
        Segment segment = leftTrajectory.segments[pathIndex];
        double leftTarget = metersToInches(segment.position);
        segment = rightTrajectory.segments[pathIndex];
        double rightTarget = metersToInches(segment.position);

        // double headingTarget = Pathfinder.r2d(segment.heading);
        // double currentHeading = Robot.drivetrain.getHeading();

        System.out.format("%f %f %f %f %f\n", timer.get(), leftDistance, leftTarget, rightDistance, rightTarget);

    }

    private static double feetToMeters(double feet) {
        return 2.54 * 12 * feet / 100;
    }

    //
    // private double getNumberOnDashboard(String name, double defaultValue) {
    // return SmartDashboard.getNumber(name, defaultValue);
    // }

    private void printInitializeMessage() {
        System.out.println("DrivePath.initialize()");
        System.out.println("Publish :" + Robot.publishPath);
    }

    private void printEndMessage() {
        System.out.println("DrivePath.end()");
    }

    private Waypoint[] calculateStraightPoints(double x) {
        return new Waypoint[] { new Waypoint(0, 0, 0), new Waypoint(x, 0, 0) };
    }

    private Waypoint[] calculateHookpoints(double x, double y){
        Waypoint[] waypoints = new Waypoint[2];
        waypoints[0] = new Waypoint(0, 0, 0);
        waypoints[1] = new Waypoint(x, y, Pathfinder.d2r(90));
        if (Robot.robotPosition ==  RIGHT_POSITION)
            return waypoints;
        else 
            return mirrorWaypoints(waypoints);
    }

    private void addPlotData() {
        if (pathIndex >= pathPoints) {
            System.out.println("Error pathIndex (" + pathIndex + ") is greater than pathPoints (" + pathIndex +")");
        return;
        }
        PathData pathData = new PathData();

        pathData.time = timer.get();
        pathData.data[0] = 12 * (Robot.drivetrain.getLeftDistance());
        pathData.data[2] = 12 * (Robot.drivetrain.getRightDistance());
        Segment leftSegment = leftTrajectory.get(pathIndex);
        Segment rightSegment = rightTrajectory.get(pathIndex);
        pathData.data[1] = metersToInches(leftSegment.position);
        pathData.data[3] = metersToInches(rightSegment.position);
        pathData.data[4] = -Robot.drivetrain.getHeading(); // Assuming the gyro is giving a value in degrees
        double th = Pathfinder.r2d(rightSegment.heading); // Should also be in degrees
        pathData.data[5] = th > 180 ? th - 360 : th; // convert to signed angle fixes problem:th 0->360 gh:-180->180
        pathDataList.add(pathData);
    }

    private static void publish(ArrayList<PathData> dataList, int traces) {
        double info[] = new double[4];
        int points = dataList.size();
        info[0] = plotCount;
        info[1] = traces;
        info[2] = points;
        info[3] = 3;

        System.out.println("Publishing Plot Data");
        // table.putValue("NewPlot", NetworkTableValue.makeDoubleArray(info));
        table.putNumber("NewPlot",plotCount); 
        table.putNumberArray("PlotParams" + plotCount, info);

        for (int i = 0; i < points; i++) {
            PathData pathData = dataList.get(i);
            double data[] = new double[traces + 2];
            data[0] = (double) i;
            data[1] = pathData.time;
            for (int j = 0; j < traces; j++) {
                data[j + 2] = pathData.data[j];
            }
            table.putNumberArray("PlotData" + i, data);
        }
        dataList.clear();
        plotCount++;
    }

    public class PathData {
        static final int DATA_SIZE = 6;
        double time = 0;
        double data[] = new double[DATA_SIZE];
    }
    

}
