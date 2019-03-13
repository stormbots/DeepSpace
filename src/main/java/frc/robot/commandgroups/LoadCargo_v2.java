/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.robot.Robot;
import frc.robot.commands.ArmPose;
import frc.robot.commands.HandPose;
import frc.robot.commands.IntakeSetPosition;
import frc.robot.commands.PassThroughPower;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.PassThrough;

public class LoadCargo_v2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LoadCargo_v2() {
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
    addSequential(new HandPose(Hand.Position.CLOSE,Hand.EJECT_POWER));
    //Start up the passthrough slowly
    addParallel(new IntakeSetPosition(100));
    addParallel(new ArmPose(41+4,-80,-90));
    //wait for both poses to be where they need to
    addSequential(new WaitForChildren());
    addSequential(new HandPose(Hand.Position.CLOSE,0));
    //start up the passthrough

    //   //if on target ()
    addSequential(new WaitCommand(0.1));
    addSequential(new ArmPose(Pose.LOAD_CARGO));
    // //Open the hand now, with a fallback to close and hold the ball if interrupted 
    addParallel(new HandPose(Hand.Position.OPEN,Hand.GRAB_POWER,  Hand.Position.CLOSE,Hand.HOLD_POWER));

    // //use the TimeOut argument of addSequential to hijack the passthrough command with a max time
    addSequential(new WaitCommand(0.1));
    addSequential(new PassThroughPower(PassThrough.LOAD_BALL_POWER),1.5);
    addParallel(new HandPose(Hand.Position.CLOSE,Hand.HOLD_POWER));

    // //Exit the loading pose safely
    addParallel(new ArmPose(41+4,-80,-90));
    addParallel(new ArmPose(41,-100,0));



    //old for reference
    // addSequential(new HandPose(Hand.Position.CLOSE,Hand.HOLD_POWER+.1));
    // addSequential(new WaitCommand(1/4));
    // addSequential(new ArmPose(Pose.LOAD_CARGO_PREP_2));
    // addSequential(new ArmPose(Pose.LOAD_CARGO_PREP));
    // addSequential(new HandPose(Hand.Position.CLOSE, Hand.HOLD_POWER));
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
