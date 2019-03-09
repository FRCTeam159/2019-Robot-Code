/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.LightSystemCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
* An example subsystem.  You can replace me with your own Subsystem.
*/
public class LightSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  SendableChooser<Integer> lightChooser = new SendableChooser<>();
  DigitalOutput dio0 = new DigitalOutput(0);
  DigitalOutput dio1 = new DigitalOutput(1);
  DigitalOutput dio2 = new DigitalOutput(2);
   DigitalOutput dio3 = new DigitalOutput(3);
  //Relay relay = new Relay(3, Direction.kForward);

  public LightSystem() {
    lightChooser.addObject("Red", 0);
    lightChooser.addDefault("Red Breath", 1);
    lightChooser.addObject("Blue", 2);
    lightChooser.addObject("Blue Breath", 3);
    lightChooser.addObject("Blue-White-Blue Synch", 4);
    lightChooser.addObject("Blue-White-Blue Opposite", 5);
    lightChooser.addObject("Random Color Blend", 6);
    lightChooser.addObject("Random Color Switch", 7);
    SmartDashboard.putData("light", lightChooser);
    SmartDashboard.putBoolean("Enable lights", false);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LightSystemCommands());
  }

  public int getLightSelection() {
    return lightChooser.getSelected();
  }

  public void setLightSelection(int sel) {
    boolean bit0 = ((sel & 1) > 0) ? true : false;
    boolean bit1 = ((sel & 2) > 0) ? true : false;
    boolean bit2 = ((sel & 4) > 0) ? true : false;
    dio0.set(bit0);
    dio1.set(bit1);
    dio2.set(bit2);
  }

  public void enableLights(boolean yes) {
     dio3.set(yes);
   /* if (yes)
      relay.set(Value.kOn);
    else
      relay.set(Value.kOff); */
  }
}
