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
import com.stormbots.closedloop.FB;
import com.stormbots.Lerp;
import frc.robot.subsystems.ArmElevator.Mode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX armMotor = new TalonSRX(12);
      public DigitalInput armLimit = new DigitalInput(0);

      double maxPos = 5000.0; //placeholder
      double minPos = 0.0;
      double targetArmPos = 0.0;
      double currentArmPos = 0.0;
      double fbGain;
      double voltageRampRate;
      double armVel = 0.0;
      double kf = 1;
      //double armAngle = 0.0;

      public Arm() {

            reset();
            voltageRampRate = 0.0;
            armMotor.configClosedloopRamp(voltageRampRate);

      }

      public Lerp armAngle = new Lerp(0, 4096/4, 0, 120); //CHANGE!!!! BAD VALUES, VERY BAD

      private boolean aHomed = false;

      private Mode mode = Mode.CLOSEDLOOP;
      
      public void setMode(Mode newMode) {
		mode = newMode;
      }

      public Mode getMode(){
            return mode;
      } 

      public void setPos(double pos){
            targetArmPos = pos;
      }

      public void reset() {
            armMotor.setSelectedSensorPosition(0, 0, 20);
      }

      public enum ArmPosition {
            MAX(5_000), MIN(0);

            double ticks = 0;
		ArmPosition(double ticks){this.ticks = ticks;}
		double ticks() {return this.ticks;};
      }

      /* public enum Mode{
            MANUAL, CLOSEDLOOP, HOMING, DISABLED
      }
      */
      

      public void updateArm(){

            currentArmPos = armMotor.getSelectedSensorPosition(0);

            switch(mode){
                  case MANUAL:

                        Clamp.clamp(targetArmPos, minPos, maxPos);
                        
                  break;

                  case CLOSEDLOOP:

                        Clamp.clamp(targetArmPos, minPos, maxPos);
                        armVel = FB.fb(targetArmPos, currentArmPos, fbGain);//kf*Math.cos(Math.toRadians(armAngle.get(currentArmPos)));//find voltage needed to keep arm level (kf),
                        //then add kf*cos(x) to output of PID loop, x=0 rad when arm is level

                  break;

                  case HOMING:
                        if(!armLimit.get()) {
                              armVel = 0;
                        }
                        else {
                              armVel = -0.3;
                        }

                  break;

                  case DISABLED:

                  break;

            }

            //manipulate our velocity
		if(!armLimit.get() && armVel <0) {
			armVel = 0;
		}
		
		//check for limit switch and reset if found
		if(!armLimit.get()) {
			aHomed = true;
			reset();
            }

            armMotor.set(ControlMode.PercentOutput, armVel);
      }


      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
