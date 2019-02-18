/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.stormbots.Clamp;
import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX elevMotor = new TalonSRX(10);
      public TalonSRX elevMotorF = new TalonSRX(11);
     
      //ublic TalonSRX elevMotor2 = new TalonSRX(11);
      DigitalInput elevLimit = new DigitalInput(2);

      //Elevator has two positions that need to be programmed in supposedly
      public static final double MAX_HEIGHT = 69;
      public static final double MIN_HEIGHT = 42;
      double elevatorHeightRestriction = MAX_HEIGHT;
      double elevatorTargetHeight = 0;
      double currentPos = 0;
      // add more positions for lvl 1,2,3 of cargo and hatches
      double kElevatorGain = 0.1;
      double elevatorPwr = 0;
      double elevatorFF = 0.2;

      /** Convert ticks to the positional height of the arm pivot, in inches */
      public Lerp elevToInches = new Lerp(0, -25_000, MIN_HEIGHT, MAX_HEIGHT); //maxElevator  -28450, minElevator -3170 

      public Elevator(){
            // bind elevator motors here
            //NOTE: We don't want ro reset normally as much of the robot operation depends
            // on initial powerup configuration
            // TRY NOT TO. reset();
            double voltageRampRate = 0.2; //old values vvvvvv
            elevMotor.configOpenloopRamp(voltageRampRate);
            elevMotor.set(ControlMode.PercentOutput,0);
            elevMotor.setSensorPhase(true);

            elevMotorF.follow(elevMotor);
      }


      private Mode mode = Mode.CLOSEDLOOP;
      private boolean homed = false;
      
      public void setMode(Mode newMode) {
		mode = newMode;
      }

      public Mode getMode(){
            return mode;
      }

      /** Convert ticks to the positional height of the arm pivot, in inches */
      public void setPosition(double pos){
            elevatorTargetHeight = pos;
      }

      /** Convert ticks to the positional height of the arm pivot, in inches */
      public double getPosition(){
            return elevToInches.get(elevMotor.getSelectedSensorPosition(0));
      }

      public void reset(){
            elevMotor.setSelectedSensorPosition(0, 0, 20);
      }

      public void set(Pose pose){
            setPosition(pose.eleHeight());
      }

      public boolean isOnTarget(double tolerance){
            return Clamp.bounded( getPosition(), elevatorTargetHeight-tolerance, elevatorTargetHeight+tolerance);
      }

      public void update(){
            
            currentPos = getPosition();
            //Use a local target copy to avoid modifying our long term target
            double target = this.elevatorTargetHeight;

            switch(mode){
                  case MANUAL:
                        //TODO : Manual mode not implemented
                  break;

                  case CLOSEDLOOP:
                        //Restrict our height based on physical limits
                        target = Clamp.clamp(target, MIN_HEIGHT, MAX_HEIGHT);
                        //Restrict our height based on current pose constraints
                        target = Clamp.clamp(target, MIN_HEIGHT, elevatorHeightRestriction);

                        //elevatorPwr = FB.fb(target, currentPos, kElevatorGain);
                        if(elevatorPwr > 0) elevatorPwr += elevatorFF;
                        else elevatorPwr += elevatorFF/3.0;
                  break;

                  case HOMING:

                        if(!elevLimit.get()) {
                              elevatorPwr = 0;
                        }
                        else {
                              elevatorPwr = -0.3;
                        }
                  break;

                  case DISABLED:
                  
                  break;
            }

            // //manipulate our velocity
		// if(!elevLimit.get() && elevatorPwr <0) {
		// 	elevatorPwr = 0;
		// }
		
		// //check for limit switch and reset if found
		// if(!elevLimit.get()) {
		// 	homed = true;
		// 	reset();
            // }

            elevMotor.set(ControlMode.PercentOutput, elevatorPwr);

            //ArmElevator.armavatorTab.add("Elevator Power", elevatorPwr);
            //ArmElevator.armavatorTab.add("Limit Touched", elevLimit.get());
            SmartDashboard.putNumber("ELevator Height (in)", currentPos);
            SmartDashboard.putNumber("Elevator Power", elevatorPwr);
            SmartDashboard.putNumber("Elevator Feed Fwd", elevatorFF);
            SmartDashboard.putNumber("Elevator FB", FB.fb(target, currentPos, kElevatorGain));
      }

      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
