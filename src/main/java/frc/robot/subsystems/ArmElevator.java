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
	/*public TalonSRX elevMotor = new TalonSRX(10);
	public TalonSRX armMotor = new TalonSRX(12);
      DigitalInput elevLimit = new DigitalInput(0);
      DigitalInput armLimit = new DigitalInput(1);
      */
      
      public Elevator elevator = new Elevator();
      public Wrist wrist = new Wrist();
      public Arm arm = new Arm();
      public static ShuffleboardTab armavatorTab = Shuffleboard.getTab("Armavator");

      //TODO: Do we want to calculate anything with these constants?
      // public final double ARM_LENGTH = 20;
      // public final double HAND_LENGTH_WITH_HATCH = 8;

	public ArmElevator() {

	}

	public enum Mode {
		MANUAL, CLOSEDLOOP, HOMING, SEPARATE, DISABLED
										
      }

	private Mode mode = Mode.SEPARATE;

	public void setMode(Mode newMode) {
            mode = newMode;
            elevator.setMode(mode);
            arm.setMode(mode);
            wrist.setMode(mode);
      }

      //TODO: Fix constants for all our poses
      // Doing things this way means we can just call 
      // set(Pose.CARGO_1) for armElevator to change everything, or for specific
      // sub-subsystems to handle individual ones. We can still do 
      // set<thing>(double) to do anything outside of this
      public enum Pose{
            // E  A  W 
            HIDE(20,-90,-90),

            CARGO_1(20,-90,0),
            CARGO_2(30,-90,0),
            CARGO_3(40,+90,0),

            HATCH_1(20,-90,0),
            HATCH_2(30,-90,0),
            HATCH_3(40,+90,0),

            LOAD_HATCH(20,-90,0),
            LOAD_CARGO(20,-80,-135);

            private double eleHeight;
            private double armAngle;
            private double wristAngle;
            Pose(double eleHeight,double armAngle,double wristAngle){
                  this.eleHeight = eleHeight;
                  this.armAngle = armAngle;
                  this.wristAngle = wristAngle;
            }
            double eleHeight() {return this.eleHeight;};
            double armAngle() {return this.armAngle;};
            double wristAngle() {return this.wristAngle;};
      }
      public Pose pose = Pose.HIDE;
      public void setPose(Pose pose){
            this.pose = pose;
            elevator.set(pose);
            wrist.set(pose);
            arm.set(pose);
      }
      public Pose getPose(){
            return this.pose;
      }

      public boolean isOnTarget(double eleTolerance,double wristTolerance,double armTolerance){
            return elevator.isOnTarget(eleTolerance)
                  && wrist.isOnTarget(wristTolerance)
                  && arm.isOnTarget(armTolerance);
      }
      public boolean isOnTarget(){ return isOnTarget(2,10,10); }


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

            if(arm.getArmAngle() < -85){
                  //our goal is to prevent snagging in the elevator
                  if( elevator.getPosition() > 30 ){
                        wrist.setLimits(0,90);
                  }else{
                        wrist.setLimits(-45,90);
                  }
            }
            else if(arm.getArmAngle() >80 ){
                  wrist.setLimits(-90,0);
            }
            else{
                  wrist.setLimits(-180,180);
            }

            // if( wrist.isOutOfBounds() ){
            //       //TODO: Attempt to block motion that would result in damage?
            //       // Likely the most viable method is to look at system setpoints and halt them 
            //       // until the arm can move out of the way 
            //     elevator.elevatorHeightRestriction = Elevator.ElevatorPosition.WRIST_SAFETY_LIMIT
            // }
            
      
            //Run all the updates
            //elevator.update();
            arm.update();
            wrist.update(arm.getArmAngle());
            
            //armavatorTab.add("Elevator Current Height", elevator.getPosition());
            //armavatorTab.add("Arm Current Angle", arm.getArmAngle());
            /*armavatorTab.add("Wrist Current Angle (Floor)", wrist.getWristAngleFromFloor());
            armavatorTab.add("Wrist Current Angle (Arm)", wrist.getWristAngleFromArm());
            armavatorTab.add("Wrist Position", wrist.wristMotor.getSelectedSensorPosition());
            */
            SmartDashboard.putNumber("Wrist Current Angle (Floor)", wrist.getWristAngleFromFloor());
            SmartDashboard.putNumber("Wrist Current Angle (Arm)", wrist.getWristAngleFromArm());
            SmartDashboard.putNumber("Wrist Position", wrist.wristMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Arm Angle", arm.getArmAngle());
            SmartDashboard.putNumber("Arm Position", arm.armMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Wrist Target Angle (Pose)", pose.wristAngle);
            
            // armavatorTab.add("Pose", this.getPose()); //causes robots don't quit
            //armavatorTab.add("Arm Pose Angle", pose.armAngle());
            //armavatorTab.add("Wrist Pose Angle", pose.wristAngle());
            //armavatorTab.add("Elevator Pose Height", pose.eleHeight());

      }


      //TODO Possibly useful for debuggin?
      // public double getHandHeight(){
      //       return elevator.getPosition() + ARM_LENGTH*Math.sin(arm.getArmAngle());
      // }

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
