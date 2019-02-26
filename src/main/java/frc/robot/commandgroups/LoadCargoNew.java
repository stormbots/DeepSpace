/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.robot.Robot;
import frc.robot.commands.ArmPose;
import frc.robot.commands.HandPose;
import frc.robot.commands.IntakeSetPosition;
import frc.robot.commands.PassThroughPower;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Hand;

public class LoadCargoNew extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LoadCargoNew() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    addSequential(new HandPose(Hand.Position.CLOSE,0));
    //Start up the passthrough slowly
    addParallel(new IntakeSetPosition(100));
    addParallel(new ArmPose(Pose.LOAD_CARGO_PREP));
    //wait for both poses to be where they need to
    addSequential(new WaitForChildren());
    //start up the passthrough

    //   //if on target ()
    addSequential(new ArmPose(Pose.LOAD_CARGO));
    //Open the hand now, with a fallback to close and hold the ball if interrupted 
    addParallel(new HandPose(Hand.Position.OPEN,Hand.GRAB_POWER,  Hand.Position.CLOSE,Hand.HOLD_POWER));

    //use the TimeOut argument of addSequential to hijack the passthrough command with a max time
    addSequential(new PassThroughPower(Robot.passThrough.LOAD_BALL_POWER),4);

    // addSequential(new WaitCommand(7.0));

    //Exit the loading pose safely
    addSequential(new HandPose(Hand.Position.CLOSE,Hand.HOLD_POWER));
    addSequential(new ArmPose(Pose.LOAD_CARGO_PREP));
    addSequential(new ArmPose(Pose.HATCH_1));


    
    //Robot.passThru.setPower(passThru.LOAD_POWER);
      //passtrough.setPower(passthrough.LOAD_POWER)
      //wait one second
      //
      //have a ball?
      //closegrabby
      //set
      //Robot.armLift.setPose(Pose.LOAD_CARGO_PREP);
      //isontarget

  }
}
