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
  double moveTime = 0.7; //NOTE: Overwritten in init to handle long arm moves
  double startTime = 0;
  Lerp timeToArmAngle;
  Lerp timeToElevatorHeight;
  Lerp timeToWristAngle;
  double armStart;
  double currentTime = 0;

  double targetArm = 0;
  double targetWrist = 0; 
  double targetEle = 0;

  public ArmPose(double eleHeight, double armAngle, double wristAngle){
    this.pose = Pose.CUSTOM;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // These line can't be instantiated by robot, doesn't like nested subsystems?
    requires(Robot.armLift);
    requires(Robot.armLift.elevator);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.arm);

    targetArm = armAngle;
    targetWrist = wristAngle; 
    targetEle = eleHeight;
  
  }

  public ArmPose(Pose pose) {
    this.pose = pose;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // These line can't be instantiated by robot, doesn't like nested subsystems?
    requires(Robot.armLift);
    requires(Robot.armLift.elevator);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.arm);

    targetArm = pose.armAngle();
    targetWrist = pose.wristAngle();
    targetEle = pose.eleHeight();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //NOTE: Setting without the setter so our smartDashboard shows the target pose,
    // but without setting the targetPositions for the systems which is what the setter does
    // This avoids some gnarly glitching where it causes the setpoint to "jump"
    Robot.armLift.pose = this.pose; 
    
    //Arm moves are violent if we exit before it's all the way up, so don't. 
    double armDelta = pose.armAngle() - Robot.armLift.arm.getArmAngle(); 
    if(armDelta > 90){
      moveTime = 0.4;
    }
    else if(armDelta < -90){
        moveTime = 0.4 ;
    }
    else{
      moveTime = 0.4;
    }
    setTimeout(2*moveTime);

    startTime = Timer.getFPGATimestamp();

    timeToArmAngle = new Lerp(0,moveTime, Robot.armLift.arm.getArmAngle(), targetArm);

    timeToElevatorHeight = new Lerp(0,moveTime, Robot.armLift.elevator.getPosition(), targetEle);

    timeToWristAngle = new Lerp(0, moveTime, Robot.armLift.wrist.getWristAngleFromFloor(), targetWrist);
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
    if( isTimedOut() ==true ) return true;
    return currentTime >= moveTime && Robot.armLift.isOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Make sure that we set the 
    if(this.pose == Pose.CUSTOM){
      Robot.armLift.setPose(targetEle, targetArm, targetWrist);
    }
    else{
      Robot.armLift.setPose(pose);
     }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
