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
import frc.robot.subsystems.ArmElevator.Mode;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX elevMotor = new TalonSRX(10);
     
      //ublic TalonSRX elevMotor2 = new TalonSRX(11);
      DigitalInput elevLimit = new DigitalInput(2);

      //Elevator has two positions that need to be programmed in supposedly
      double maxPos = 10000; // placeholder
      double minPos = 0;
      double elevatorPos = 0;
      double currentPos = 0;
      // add more positions for lvl 1,2,3 of cargo and hatches
      double fbGain;
      double elevatorVel = 0;

      public Lerp elevLerp = new Lerp(-1, 1, minPos, maxPos);



      public Elevator(){

            // bind elevator motors here
            reset();
            double voltageRampRate = 0.2; //old values vvvvvv
		elevMotor.configOpenloopRamp(voltageRampRate, 30);

      }


      private Mode mode = Mode.MANUAL;
      private boolean eHomed = false;
      
      public void setMode(Mode newMode) {
		mode = newMode;
      }

      public Mode getMode(){
            return mode;
      }

      public void setPos(double pos){
            elevatorPos = pos;
      }

      public void reset(){
            elevMotor.setSelectedSensorPosition(0, 0, 20);
      }

      public enum ElevatorPosition {
            MAX(10_000), MIN(0);

            double ticks = 0;
		ElevatorPosition(double ticks){this.ticks = ticks;}
		double ticks() {return this.ticks;};
      }

      public void updateElevator(){
            
            currentPos = elevMotor.getSelectedSensorPosition(0);

            switch(mode){
                  case MANUAL:
                  /*
                        Clamp.clamp(elevatorPos, minPos, maxPos);
                        elevatorVel = FB.fb(elevatorPos, currentPos, fbGain);
                        */
                  break;

                  case CLOSEDLOOP:
                        Clamp.clamp(elevatorPos, minPos, maxPos);
                        elevatorVel = FB.fb(elevatorPos, currentPos, fbGain);
                  break;

                  case HOMING:

                        if(!elevLimit.get()) {
                              elevatorVel = 0;
                        }
                        else {
                              elevatorVel = -0.3;
                        }
                  break;

                  case DISABLED:
                  
                  break;
            }

            //manipulate our velocity
		if(!elevLimit.get() && elevatorVel <0) {
			elevatorVel = 0;
		}
		
		//check for limit switch and reset if found
		if(!elevLimit.get()) {
			eHomed = true;
			reset();
            }
            elevMotor.set(ControlMode.PercentOutput, elevatorVel);
      }

      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
