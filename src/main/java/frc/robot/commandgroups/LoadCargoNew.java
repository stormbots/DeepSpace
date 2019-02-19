/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.ArmPose;
import frc.robot.commands.HandGrab;
import frc.robot.commands.IntakeSetPosition;
import frc.robot.commands.PassThroughPower;
import frc.robot.subsystems.ArmElevator.Pose;

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
    addSequential(new HandGrab(true));
    addSequential(new PassThroughPower(0.2));
    addSequential(new IntakeSetPosition(100));
    addSequential(new ArmPose(Pose.LOAD_CARGO_PREP));
    addParallel(new PassThroughPower(Robot.passThrough.LOAD_BALL_POWER));

    //   //if on target ()
    addSequential(new ArmPose(Pose.LOAD_CARGO));
    //   //if on target
    addSequential(new WaitCommand(7.0));

    // addSequential(new HandGrab(false));
    // addParallel(new HandPower(Robot.hand.GRAB_POWER));
    // addSequential(new WaitCommand(1.5));

    // addSequential(new HandGrab(true));

    // addSequential(new ArmPose(Pose.LOAD_CARGO_PREP));
    // addSequential(new ArmPose(Pose.HATCH_1));

    
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
