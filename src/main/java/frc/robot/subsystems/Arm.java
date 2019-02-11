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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX armMotor = new TalonSRX(12);

      double targetArmPos = 0.0;
      double currentArmPos = 0.0;
      double armPower = 0.0;
      
      /** Saved from the update function to enable tracking based on the floor angle */
      double armAngle = 0.0;

      double kArmGain = 0.004;
      double kArmFF = 0;

      public static final double MAX_ANGLE = 90.0;
      public static final double MIN_ANGLE = -90.0;

      public Arm() {
            // NOTE: We cannot reset the sensors, as we have no limit switches
            // to reset them against. So, instead we need to ensure robot is in a known
            // state on powerup, and not reset sensors upon code changes.
            //NO GOOD. reset();

            //as a test mode thing, we could potentially reset this using
            //current limited motors to force it back into nominal position


            armMotor.setSensorPhase(true);
            armMotor.configOpenloopRamp(0.2); 

            //Configure are current restrictions
            //TODO: Set much higher. These limits are really low for bringup safety. 
            //https://www.chiefdelphi.com/t/talon-srx-current-limiting-behaviour/164074
            armMotor.configPeakCurrentLimit(5, 10); // 35 A 
            armMotor.configPeakCurrentDuration(200, 10); // 200ms
            armMotor.configContinuousCurrentLimit(4, 10); // 30A
            armMotor.enableCurrentLimit(true); // turn it on


      }

      /** Specified by 4096 ticks per rotation, with a 54:18 gear ratio */
      public Lerp armToDegrees = new Lerp(0, 4096*(54.0/18.0), -90, -90+360);

      private Mode mode = Mode.CLOSEDLOOP;
      
      public void setMode(Mode newMode) {
		mode = newMode;
      }

      public Mode getMode(){
            return mode;
      } 

      public void setAngle(double pos){
            targetArmPos = pos;
      }

      public double getArmAngle(){
            return armToDegrees.get(armMotor.getSelectedSensorPosition());
      }
      public boolean isOnTarget(double tolerance){
            return Clamp.bounded( getArmAngle(), targetArmPos-tolerance, targetArmPos+tolerance);
      }

      public void set(Pose pose){
            setAngle(pose.armAngle());
      }

    

      public void update(){
            currentArmPos = getArmAngle();
            // Create a shadow variable in local scope to avoid incorrectly altering our target
            // This is needed as we may have to constrain the target due to dynamic influences,
            // which when changed should result in a change of our actual position
            targetArmPos = this.targetArmPos; 
            
            switch(mode){
                  case MANUAL:
                        //TODO: This clamp is a closed loop affecting operation,
                        //and doesn't impact anything
                        //Clamp.clamp(targetArmPos, MIN_ANGLE, MAX_ANGLE);
                  break;

                  case CLOSEDLOOP:

                        targetArmPos = Clamp.clamp(targetArmPos, MIN_ANGLE, MAX_ANGLE);

                        armPower = FB.fb(targetArmPos, currentArmPos, kArmGain)+
                              kArmFF*Math.cos(Math.toRadians(armToDegrees.get(currentArmPos))
                              );
                        //TODO find voltage needed to keep arm level (kArmFF),

                  break;

                  case HOMING:
                        //NOTE: We do not actually have limit switches for this.
                        //Instead, we would need to trust gravity or initial boot-up position
                        // if(!armLimit.get()) {
                        //       armPower = 0;
                        // }
                        // else {
                        //       armPower = -0.3;
                        // }
                  break;

                  case DISABLED:
                        //TODO: Disabled should set all power variables to zero
                        armPower = 0;
                  break;

            }

            //Check for soft limits
            if(armPower > 0 && currentArmPos > MAX_ANGLE) armPower = 0;
            if(armPower < 0 && currentArmPos < MIN_ANGLE) armPower = 0;

            armMotor.set(ControlMode.PercentOutput, armPower);
      }


      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}