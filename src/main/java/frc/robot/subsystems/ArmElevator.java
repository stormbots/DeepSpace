/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class ArmElevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
      
      public Elevator elevator = new Elevator();
      public Wrist wrist = new Wrist();
      public Arm arm = new Arm();
      public static ShuffleboardTab armavatorTab = Shuffleboard.getTab("Armavator");

      @Override
      public void periodic(){
            SmartDashboard.putNumber("Wrist/Current Angle (Floor)", wrist.getWristAngleFromFloor());
            // SmartDashboard.putNumber("Wrist/Current Angle (Arm)", wrist.getWristAngleFromArm());
            SmartDashboard.putNumber("Wrist/Encoder", wrist.wristMotor.getSelectedSensorPosition());
            // SmartDashboard.putNumber("Wrist/Target Angle", pose.wristAngle);

            SmartDashboard.putNumber("Arm/Current Angle", arm.getArmAngle());
            SmartDashboard.putNumber("Arm/Encoder", arm.armEncoder.getPosition());
            // SmartDashboard.putNumber("Arm/Target Angle", pose.armAngle);

            // SmartDashboard.putNumber("Elevator/Target Height", pose.eleHeight);
            SmartDashboard.putNumber("Elevator/Height", elevator.getPosition());
            SmartDashboard.putNumber("Elevator/Encoder", elevator.elevMotor.getSelectedSensorPosition());

            SmartDashboard.putString("Wrist/Pose", pose.toString());
            SmartDashboard.putString("Arm/Pose", pose.toString());
            SmartDashboard.putString("Elevator/Pose", pose.toString());

      }

	public ArmElevator() {
            this.setPose(pose.STARTUP);
      }
      
	public void robotInit() {
            arm.robotInit();
            elevator.robotInit();
            wrist.robotInit();
	}

	public enum Mode {
		MANUAL, CLOSEDLOOP, HOMING, SEPARATE, DISABLED
										
      }

	private Mode mode = Mode.SEPARATE;

	public void setMode(Mode newMode) {
            mode = newMode;
            elevator.setMode(mode);
            //arm.setMode(mode);
            //wrist.setMode(mode);
      }

      //TODO: Fix constants for all our poses
      // Doing things this way means we can just call 
      // set(Pose.CARGO_1) for armElevator to change everything, or for specific
      // sub-subsystems to handle individual ones. We can still do 
      // set<thing>(double) to do anything outside of this
      public enum Pose{
            // Elevator bottom pos 41
            // E  A  W 
            HIDE(41,-100,-155), //MAR 6 practice bot
            // HOMING(),

            STARTUP(41,-90,0), //TODO
            CUSTOM(41,-90,0), //TODO

            CARGO_1(45+2,-90,0),
            CARGO_2(41+1,+33,0), //not sure elev height, might have to move arm
            CARGO_3(65-3,+90,0),

            CARGO_SHIP(69+3,-90,-30),

            HATCH_1(41,-90,0-3),
            HATCH_2(69+2,-90,0),
            HATCH_3(58,+90,0),

            HIDE_1(41+4,-80.0 + 10, 0),
            HIDE_2(41+4,-80.0 + 10,-90.0-30),

            LOAD_HATCH(42,-90,0),
            // LOAD_CARGO(42,-59,-150); //ANCIENT practice bot, arm used to be -70, wrist angle was -135
            LOAD_CARGO_PREP(45,-53+10,0),
            LOAD_CARGO_PREP_2(45, -50+10, -166),
            // LOAD_CARGO(41,-53,-150); /MAR04 Works on practice bot. Usually works on Comp bot
            LOAD_CARGO(41,-65 - 10,-161 + 10-10); //MAR04 Should work but untested on comp bot
            //LOAD_CARBO(41, -75, -150)
            //LOAD_CARGO(41,-67,-156); //MAR05 PRactice bot maybe?

            private double eleHeight;
            private double armAngle;
            private double wristAngle;
            Pose(double eleHeight,double armAngle,double wristAngle){
                  this.eleHeight = eleHeight;
                  this.armAngle = armAngle;
                  this.wristAngle = wristAngle;
            }
            public double eleHeight() {return this.eleHeight;};
            public double armAngle() {return this.armAngle;};
            public double wristAngle() {return this.wristAngle;};
            
            // public String toString(){
            //       return String.format("Pose(E:%3.2f A:%3.2f W:%3.2f)",this.eleHeight,this.armAngle,this.wristAngle);
            // }
      }
      public Pose pose = Pose.STARTUP;
      public void setPose(Pose pose){
            this.pose = pose;
            elevator.set(pose);
            wrist.set(pose);
            arm.set(pose);
      }

      public void setPose(double eleHeight, double armAngle, double wristAngle){
            this.pose = Pose.CUSTOM;
            elevator.setPosition(eleHeight);
            wrist.setTargetAngleFromFloor(wristAngle);
            arm.setAngle(armAngle);
      }

      public Pose getPose(){
            return this.pose;
      }

      public boolean isOnTarget(double eleTolerance,double wristTolerance,double armTolerance){
            return elevator.isOnTarget(eleTolerance)
                  && wrist.isOnTarget(wristTolerance)
                  && arm.isOnTarget(armTolerance);
      }
      public boolean isOnTarget(){ return isOnTarget(2,7,7); } 


      public Mode getMode(){
            return mode;
      }

      public void update(){

            switch(mode){
                  case MANUAL:
                        //elevator.setMode(ElevMode.MANUAL);
                        //arm.setMode(ArmMode.MANUAL);

                  break;

                  case CLOSEDLOOP:
                        

                  break;

                  case HOMING:
                        

                  break;

                  case SEPARATE:

                  break;

                  case DISABLED:
                        
                  break;
            }

            //when 
            //TODO: We need to dynamically update and/or cap any mins/maxes to avoid damaging collisions
            // Try to prevent elevator movements from snagging the hand in various regions
            // This is going to be fairly difficult to get right.
            // This should be considered a "last stop" effort to protect from damage, not an optimized sequence.
            // Scenarios to watch for 
            // When arm target/current is swinging upward, we can't let the arm be pointing to the robot
            // When the arm is pointing straight down, we 
            // When the hand is in side the robot for ball pickup, we can't

            // if(arm.getArmAngle() < -85){
            //       //our goal is to prevent snagging in the elevator
            //       if( elevator.getPosition() > 30 ){
            //             wrist.setLimits(0,90);
            //       }else{
            //             wrist.setLimits(-45,90);
            //       }
            // }
            // else if(arm.getArmAngle() >80 ){
            //       wrist.setLimits(-90,0);
            // }
            // else{
            //       wrist.setLimits(-180,180);
            // }

            // if( wrist.isOutOfBounds() ){
            //       //TODO: Attempt to block motion that would result in damage?
            //       // Likely the most viable method is to look at system setpoints and halt them 
            //       // until the arm can move out of the way 
            //     elevator.elevatorHeightRestriction = Elevator.ElevatorPosition.WRIST_SAFETY_LIMIT
            // }

            // if(getCurrentCommandName().isBlank() && isOnTarget(2,10,10)){
            //       SmartDashboard.putBoolean("Elevator/ManualPose",true);
            //       switch(pose){
            //             case LOAD_CARGO_PREP:
            //             setPose(Pose.HATCH_1);
            //             break;
            //             case LOAD_CARGO_PREP_2:
            //             setPose(Pose.LOAD_CARGO_PREP);
            //             break;
            //             // case STARTUP:
            //       }
            // }
      

            //Run all the updates
            wrist.update(arm.getArmAngle());
            elevator.update(wrist.getWristAngleFromFloor(), wrist.targetWristToFloorAngle);
            arm.update(wrist.wristPower);
      }

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
