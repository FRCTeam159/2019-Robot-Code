package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Cameras extends Subsystem {

	public void initDefaultCommand() {
		
	}

	public Cameras() {
		CameraServer server = CameraServer.getInstance();
		UsbCamera driverCamera = server.startAutomaticCapture("Driver", 0);
//		UsbCamera elevatorCamera = server.startAutomaticCapture("Elevator", 1);
		//driverCamera.setFPS(25);
		//driverCamera.setResolution(320,240);
	}
}
