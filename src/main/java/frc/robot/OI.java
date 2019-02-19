/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commandgroups.LoadCargoNew;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Chassis.Gear;
// import frc.robot.commands.ArmPose;
// import frc.robot.subsystems.ArmElevator;
// import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public static Joystick driveStick = new Joystick(0);
  public Joystick rightStick = new Joystick(1);
  public Joystick leftStick = new Joystick(2);

  // This should be a really nice way to set up buttons to quickly switch around to various poses
  // Most notably, the old joysticks actually have 6 buttons on the left side, which is almost
  // perfect for Cargo + Hatch levels 1-3, as well has having a few spare buttons that could be OK for hatches
  // I suspect just holding the main trigger would serve well for quick pickup of the cargo,
  // and another button would serve for loading

  //rightStick focuses on cargo, leftStick focuses on hatches

  //Right
  public JoystickButton poseCargo1 = new JoystickButton(rightStick,4);
  public JoystickButton poseCargo2 = new JoystickButton(rightStick,2);
  public JoystickButton poseCargo3 = new JoystickButton(rightStick,5);
  public JoystickButton intakeCargo = new JoystickButton(rightStick,1);
  public JoystickButton armDown = new JoystickButton(rightStick,6); //Pulls arm in to keep it safe
  public JoystickButton climbSequence = new JoystickButton(rightStick,8);

  //Might need this, but moving the arm to grab a ball from the intake may not be possible as a sequence
  public JoystickButton grabCargo = new JoystickButton(rightStick,3); 

  //Left
  public JoystickButton poseHatch1 = new JoystickButton(leftStick,4);
  public JoystickButton poseHatch2 = new JoystickButton(leftStick,2);
  public JoystickButton poseHatch3 = new JoystickButton(leftStick,5);
  public JoystickButton loadHatch = new JoystickButton(leftStick,3);
  public JoystickButton handGrab = new JoystickButton(leftStick,1);
  public JoystickButton handSuckIn = new JoystickButton(leftStick,6);
  public JoystickButton handSpitOut = new JoystickButton(leftStick,7);
  public JoystickButton autoLineup = new JoystickButton(leftStick,8);

  //NOT PROPERLY ASSIGNED
  public JoystickButton pixyfollower = new JoystickButton(driveStick, 5);
  public JoystickButton shifter = new JoystickButton(driveStick, 6);


  //This switches functionality between a separate lineup, arm position, and drive forward delivery system
  //vs. one that runs the whole sequence of events with one button (one for each position)
  public JoystickButton switchDeliverMode = new JoystickButton(leftStick,11);



  

  public OI(){

    poseCargo1.whenPressed(new ArmPose(Pose.CARGO_1));
    poseCargo2.whenPressed(new ArmPose(Pose.CARGO_2));
    poseCargo3.whenPressed(new ArmPose(Pose.CARGO_3));
    //intakeCargo.whenPressed(new ArmPose(Pose.LOAD_CARGO));
    intakeCargo.whenPressed(new LoadCargoNew());
    //armDown.whenPressed(new ArmPose(Pose.HIDE));

    poseHatch1.whenPressed(new ArmPose(Pose.HATCH_1));
    poseHatch2.whenPressed(new ArmPose(Pose.HATCH_2));
    poseHatch3.whenPressed(new ArmPose(Pose.HATCH_3));
    //loadHatch.whenPressed(new ArmPose(Pose.LOAD_HATCH));

    //autoLineup.whileHeld(new ChassisPixyDrive());

    handSuckIn.whileHeld(new HandPower(Robot.hand.GRAB_POWER,Robot.hand.HOLD_POWER));
    handSpitOut.whileHeld(new HandPower(Robot.hand.EJECT_POWER));
    handGrab.toggleWhenPressed(new HandGrab());

    grabCargo.whileHeld(new IntakeGrabBall());

    //climbSequence.whileHeld(new RobotGrabHab());

    pixyfollower.whileHeld(new ChassisPixyDrive());
    
    // Press+Release creates a "hold" behaviour without special isFinished() conditions
    shifter.whenPressed(new ChassisShift(Gear.HIGH));
    shifter.whenReleased(new ChassisShift(Gear.LOW));
    climbSequence.whileHeld(new RobotGrabHab(5));


  }
  


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

  // public double getElevatorPos(){
  //  return Lerp.lerp( rightStick.getRawAxis(3) ,1,-1, Robot.armLift.elevator.MIN_HEIGHT, Robot.armLift.elevator.MAX_HEIGHT);
  // }

  // public double getArmPos(){
  //   return Lerp.lerp( rightStick.getRawAxis(3) ,1,-1, Robot.armLift.arm.MIN_ANGLE, Robot.armLift.arm.MAX_ANGLE);
  // }
  

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

  


  public static double getDriveFwdL() {
    //return driveStick.getRawAxis(1);
    return driveStick.getRawAxis(1);//*Math.abs(driveStick.getRawAxis(1)); // CADEN's drive
    // make the output "-" for Zeus
  }

  public static double getDriveFwdR() {
    //return driveStick.getRawAxis(3);
    return driveStick.getRawAxis(3);//*Math.abs(driveStick.getRawAxis(3));  // NORMAL PEOPLE's drive
    // make the output "-" for Zeus
  }

  public static double getDriveSideL() {
    return driveStick.getRawAxis(0);//*Math.abs(driveStick.getRawAxis(0)); // NORMAL PEOPLE's drive
    // make the output "-" for Zeus
  }

  public static double getDriveSideR() {
    return driveStick.getRawAxis(2);//*Math.abs(driveStick.getRawAxis(2)); // CADEN's drive
  // make the output "-" for Zeus
  }
}
