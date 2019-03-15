/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.Clamp;
import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      // public TalonSRX armMotor = new TalonSRX(12);
      public CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);
      public CANEncoder armEncoder = new CANEncoder(armMotor);
      public DigitalInput armLimit = new DigitalInput(RobotMap.ArmLimitSwitch);

      FakeGyro gyro = new FakeGyro();

      double targetArmPos = 0.0;
      double currentArmPos = 0.0;
      double armPower = 0.0;
      
      /** Saved from the update function to enable tracking based on the floor angle */
      double armAngle = 0.0;

      // double kArmGain = 0.027;
      double kArmGain; 
      double kWristFF;

      // double kArmFF = 0.3;
      double kArmFF;

      public static final double MAX_ANGLE = 90.0;
      public static       double MIN_ANGLE = -115.0;

      boolean lastSwitch = false;

      @Override
      public void periodic(){
            SmartDashboard.putString("Arm/Command", getCurrentCommandName());
            SmartDashboard.putData("Arm/Position", gyro.set(getArmAngle()));
            SmartDashboard.putBoolean("Arm/LimitSwitch", isLimitPressed());
      }

      public Arm() {
            // NOTE: We cannot reset the sensors, as we have no limit switches
            // to reset them against. So, instead we need to ensure robot is in a known
            // state on powerup, and not reset sensors upon code changes.
            //NO GOOD. reset();

            //as a test mode thing, we could potentially reset this using
            //current limited motors to force it back into nominal position

            // armMotor.configOpenloopRamp(0.2); 
            armMotor.setOpenLoopRampRate(0.1);

            //configure are current restrictions
            //TODO: Set much higher. These limits are really low for bringup safety. 
            //https://www.chiefdelphi.com/t/talon-srx-current-limiting-behaviour/164074

            armMotor.setSmartCurrentLimit(30, 10, 5700/10);

            armMotor.setIdleMode(IdleMode.kCoast);

      }

      /** Runs on robot boot after network/SmartDashboard becomes available */
      public void robotInit(){
            if(Robot.isCompbot){

                  armMotor.setInverted(true);
                  kArmGain = 0.025;
                  kArmFF = 0.2; 
                  kWristFF = 0.02;
                  // kWristFF = 0.0;
            }
            else{
                  kWristFF = 0.9;
                  kArmFF = 0.5;
                  // kArmGain = 0.1;//too high
                  // kArmGain = 0.042; //too low 
                  kArmGain = 0.09;
                  // armMotor.setInverted(false);
                  MIN_ANGLE = -115.0;
                  kWristFF = 0.9;
            }
      }

      /** Specified by 4096 ticks per rotation, with a 54:18 gear ratio */
      public Lerp armToDegrees = new Lerp(0, 93, -90, 90);

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
            return armToDegrees.get(armEncoder.getPosition());
            
      }

      public boolean isOnTarget(double tolerance){
            return Clamp.bounded( getArmAngle(), targetArmPos-tolerance, targetArmPos+tolerance);
      }

      public void set(Pose pose){
            setAngle(pose.armAngle());
      }

      public void setEncoderAngle(double angle){
            armEncoder.setPosition((int)armToDegrees.getReverse(angle));
      }

      public boolean isLimitPressed(){
            return armLimit.get() == false;
      }

      public void setPower(double power){
            armPower = power;
      }
    

      public void update(double wristPower){
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

                        armPower = FB.fb(targetArmPos, currentArmPos, kArmGain);

                        if(armPower >0 )  armPower += kArmFF*Math.cos(Math.toRadians(currentArmPos));
                        armPower += wristPower*-kWristFF;

                        SmartDashboard.putNumber("Arm/Output FB", FB.fb(targetArmPos, currentArmPos, kArmGain));
                        SmartDashboard.putNumber("Arm/Output FF",  kArmFF*Math.cos(Math.toRadians(currentArmPos)));
                        SmartDashboard.putNumber("Arm/Output Wrist FF",  wristPower*-kWristFF);

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

            //if switch is pressed && lastSwitch was not pressed
                  //know we're at angle X
            //lastswitch = switch
            //TODO: check on old load to make sure it's consistent and non-breaking;
            if(isLimitPressed() != lastSwitch){ //old "working version"
            // if(isLimitPressed() != lastSwitch && lastSwitch == true){ //should fix for slop
                  setEncoderAngle(-87);
            }
            lastSwitch = isLimitPressed();


            //Check for soft limits
            if(armPower > 0 && currentArmPos > MAX_ANGLE) armPower = 0;
            if(armPower < 0 && currentArmPos < MIN_ANGLE) armPower = 0;
            // armPower = Clamp.clamp(armPower, -0.05, 0.05);

            armMotor.set(armPower);
            SmartDashboard.putNumber("Arm/Output Total", armPower);
            SmartDashboard.putNumber("Arm/Amps", armMotor.getOutputCurrent());
      }

      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}