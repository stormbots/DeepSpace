/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.ArmPose;
import frc.robot.subsystems.ArmElevator;
import frc.robot.subsystems.ArmElevator.Pose;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick clyde = new Joystick(0);

  // This should be a really nice way to set up buttons to quickly switch around to various poses
  // Most notably, the old joysticks actually have 6 buttons on the left side, which is almost
  // perfect for Cargo + Hatch levels 1-3, as well has having a few spare buttons that could be OK for hatches
  // I suspect just holding the main trigger would serve well for quick pickup of the cargo,
  // and another button would serve for loading
  /*
  public JoystickButton poseCargo1 = new JoystickButton(clyde,7);
  public OI(){
    poseCargo1.whenPressed(new ArmPose(Pose.CARGO_1));
  }
  //*/


  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  public double getElevatorPos(){
   return Lerp.lerp( clyde.getRawAxis(3) ,1,-1, Robot.armLift.elevator.MIN_HEIGHT, Robot.armLift.elevator.MAX_HEIGHT);
  }

  public double getArmPos(){
    return Lerp.lerp( clyde.getRawAxis(3) ,1,-1, Robot.armLift.arm.MIN_ANGLE, Robot.armLift.arm.MAX_ANGLE);
  }
  

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());

}
