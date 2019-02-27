/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.Clamp;
import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Pose;

public class ArmPose extends Command {
  Pose pose;
  double moveTime = 0.5;
  double startTime = 0;
  Lerp timeToArmAngle;
  Lerp timeToElevatorHeight;
  Lerp timeToWristAngle;
  double armStart;
  double currentTime;

  public ArmPose(Pose pose) {
    this.pose = pose;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // These line can't be instantiated by robot, doesn't like nested subsystems?
    requires(Robot.armLift);
    requires(Robot.armLift.elevator);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.arm);
    currentTime = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Avoid catastrophic pose changes if we're in the robot
    if(Robot.armLift.pose==Pose.LOAD_CARGO) pose = Pose.LOAD_CARGO_PREP;
    Robot.armLift.setPose(pose); //TODO: needed to set for dashboard, now or later?

    startTime = Timer.getFPGATimestamp();

    timeToArmAngle = new Lerp(0,moveTime, Robot.armLift.arm.getArmAngle(), pose.armAngle());

    timeToElevatorHeight = new Lerp(0,moveTime, Robot.armLift.elevator.getPosition(), pose.eleHeight());

    timeToWristAngle = new Lerp(0, moveTime, Robot.armLift.wrist.getWristAngleFromFloor(), pose.wristAngle());
    }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentTime = Timer.getFPGATimestamp() - startTime;
    currentTime = Clamp.clamp(currentTime, 0, moveTime);

    //Arm Stuff
    Robot.armLift.arm.setAngle(timeToArmAngle.get(currentTime));
    //Elevator Stuff
    Robot.armLift.elevator.setPosition(timeToElevatorHeight.get(currentTime));
    //Wrist stuff
    Robot.armLift.wrist.setTargetAngleFromFloor(timeToWristAngle.get(currentTime));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return currentTime > moveTime && Robot.armLift.isOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armLift.setPose(pose);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
