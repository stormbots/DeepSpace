/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.ArmHoming;
import frc.robot.commands.ArmPose;
import frc.robot.commands.HandPose;
import frc.robot.commands.IntakeHoming;
import frc.robot.commands.PassThroughPower;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class DefenseModeEngage extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DefenseModeEngage() {
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
    addParallel(new PassThroughPower(-0.3),2);
    addParallel(new HandPose(Hand.Position.CLOSE,Hand.EJECT_POWER,  Hand.Position.CLOSE,Hand.OFF),0.2);
    // addParallel(new IntakeHoming());
    //                        E     A     W
    addSequential(new ArmPose(Pose.HIDE_1));
    addSequential(new ArmPose(Pose.HIDE_2));
    addSequential(new ArmPose(Pose.HIDE));
    addSequential(new ArmHoming());
  }
}
