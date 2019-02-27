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

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX armMotor = new TalonSRX(12);

      FakeGyro gyro = new FakeGyro();

      double targetArmPos = 0.0;
      double currentArmPos = 0.0;
      double armPower = 0.0;
      
      /** Saved from the update function to enable tracking based on the floor angle */
      double armAngle = 0.0;

      // double kArmGain = 0.027;
      // double kArmGain = 0.035; //comp bot
      double kArmGain = 0.042;

      
      // double kArmFF = 0.3;
      double kArmFF = 0.3; // See RobotInit for practicebot

      public static final double MAX_ANGLE = 90.0;
      public static final double MIN_ANGLE = -90.0;

      @Override
      public void periodic(){
            SmartDashboard.putData("Arm/Position",gyro.set(getArmAngle()));
      }

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
            armMotor.configPeakCurrentLimit(10, 10); // 35 A 
            armMotor.configPeakCurrentDuration(200, 10); // 200ms
            armMotor.configContinuousCurrentLimit(8, 10); // 30A
            armMotor.enableCurrentLimit(true); // turn it on
      }

      /** Runs on robot boot after network/SmartDashboard becomes available */
      public void robotInit(){
            if(Robot.isCompbot){
                  armMotor.setInverted(true);
            }
            else{
                  kArmFF = 0.5;
                  // kArmGain = 0.1;//too high
                  // kArmGain = 0.042; //too low 
                  kArmGain = 0.08;
                  armMotor.setInverted(false);
            }
      }

      /** Specified by 4096 ticks per rotation, with a 54:18 gear ratio */
      public Lerp armToDegrees = new Lerp(0, 3112.0, -90, 0);

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
            SmartDashboard.putString("arm/command",getCurrentCommandName());
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
                              kArmFF*Math.cos(Math.toRadians(currentArmPos)
                              );

                        SmartDashboard.putNumber("Arm/Output FB", FB.fb(targetArmPos, currentArmPos, kArmGain));
                        SmartDashboard.putNumber("Arm/Output FF",  kArmFF*Math.cos(Math.toRadians(currentArmPos)));

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
                        armPower = 0;
                  break;

            }

            //Check for soft limits
            if(armPower > 0 && currentArmPos > MAX_ANGLE) armPower = 0;
            if(armPower < 0 && currentArmPos < MIN_ANGLE) armPower = 0;
            //armPower = Clamp.clamp(armPower, -0.2, 0.2);
            armMotor.set(ControlMode.PercentOutput, armPower);
            SmartDashboard.putNumber("Arm/Output Total", armPower);
            SmartDashboard.putNumber("Arm/Amps", armMotor.getOutputCurrent());
      }


      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
