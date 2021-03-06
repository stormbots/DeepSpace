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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.stormbots.Clamp;
import com.stormbots.Lerp;
import com.stormbots.closedloop.MiniPID;

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
public class Wrist extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX wristMotor = new TalonSRX(13);

      public MiniPID pidWrist = new MiniPID(0.012, 0.001*0.0, 0.0);
      //public MiniPID pidWrist = new MiniPID(1.0/1350.0*6, 1.0/4000.0*0.0, 1.0/100000.0); //these values are pretty good
      //public MiniPID pidWrist = new MiniPID(1.0/35.0, 0.0, 0.0004/2.0*0.0);
      //NOTE: Curremt implementation uses only floor-relative angles
      FakeGyro gyro = new FakeGyro();
      double targetWristToFloorAngle = 0.0;

      public double wristPower = 0.0;
      double armPosition = 0.0; // Used for floor-relative calculations

      //double kWristGain = 0.005;
      //double kWristGain = 0.006;
      //double kWristGain = 0.0065;
      //double kWristGain = 0.007;
      //double kWristGain = 0.01;
      double kWristFF = 0.13;

      // Physical limits that apply to the wrist at all times
      public static final double MAX_ANGLE_TO_ARM = 90.0;
      public static final double MIN_ANGLE_TO_ARM = -90.0;

      // Physical limits that may be changed based on dynamic constraints and robot positions
      double maxAngleToArm = 90.0;
      double minAngleToArm = -90.0;
      boolean isHomed = false;
      public static double HOME_OUTPUT_POWER_MIN = 0.09;
      Pose wristPose = Pose.STARTUP;

      @Override
      public void periodic(){
            SmartDashboard.putData("Wrist/Position", gyro.set(getWristAngleFromFloor()));
            SmartDashboard.putString("Wrist/Command", getCurrentCommandName());
            SmartDashboard.putNumber("Wrist/Output Total", wristPower);
            SmartDashboard.putNumber("Wrist/Amps", wristMotor.getOutputCurrent());
            SmartDashboard.putBoolean("Wrist/Homed?", isHomed());
            SmartDashboard.putString("Wrist/Current Pose", this.wristPose.toString());
            SmartDashboard.putBoolean("Wrist/LimitSwitch", isLimitPressed());

      }

      public Wrist() {
            // NOTE: We cannot reset the sensors, as we have no limit switches
            // to reset them against. So, instead we need to ensure robot is in a known
            // state on powerup, and not reset sensors upon code changes.
            //NO GOOD. reset();

            //as a test mode thing, we could potentially reset this using
            //current limited motors to force it back into nominal position
            //reset();

            wristMotor.configOpenloopRamp(0.2);
            wristMotor.setSensorPhase(true);
            
            //Configure are current restrictions
            //TODO: Set much higher. These limits are really low for bringup safety. 
            //https://www.chiefdelphi.com/t/talon-srx-current-limiting-behaviour/164074
            wristMotor.configPeakCurrentLimit(10, 10); // 35 A 
            wristMotor.configPeakCurrentDuration(200, 10); // 200ms
            wristMotor.configContinuousCurrentLimit(6, 10); // 30A
            wristMotor.enableCurrentLimit(true); // turn it on

            //Avoid glitching on code updates/restarts by defaulting to hold whatever position we start in
            targetWristToFloorAngle = getWristAngleFromFloor();
            pidWrist.setOutputLimits(-1+kWristFF,1-kWristFF);
            pidWrist.setMaxIOutput(0.2);

            wristMotor.configForwardLimitSwitchSource(
                  LimitSwitchSource.Deactivated, 
                  LimitSwitchNormal.NormallyOpen);
            wristMotor.configReverseLimitSwitchSource(
                  LimitSwitchSource.Deactivated, 
                  LimitSwitchNormal.NormallyOpen);
      }

      /** Runs on robot boot after network/SmartDashboard becomes available */      
      public void robotInit(){
            if(Robot.isCompbot){
                  wristMotor.setInverted(true);
                  kWristFF = 0.11;
                  HOME_OUTPUT_POWER_MIN = 0.09;
            }
            else{
                  pidWrist = new MiniPID(1.0/1350.0*6, 1.0/20000.0, 1.0/100000.0);
                  pidWrist.setMaxIOutput(0.15);
                  kWristFF = 0.11;
                  wristMotor.setInverted(true);
                  wristMotor.setSensorPhase(false);
                  // HOME_OUTPUT_POWER_MIN = 0.12; //old usable
                  // HOME_OUTPUT_POWER_MIN = 0.09; //low power, but insufficient for matches
                  HOME_OUTPUT_POWER_MIN = 0.11;
            }
      }
      /** Specified by 4096 ticks per rotation, with a 42:24 gear ratio */
      // public Lerp wristToDegrees = new Lerp(0, 4096*(42.0/24.0), 0, 360);
      public Lerp wristToDegrees = new Lerp(0, 2395.0, 0, 90);

      private Mode mode = Mode.CLOSEDLOOP;
      
      public void setMode(Mode newMode) {
		mode = newMode;
      }

      public Mode getMode(){
            return mode;
      } 

      public void setTargetAngleFromFloor(double pos){
            targetWristToFloorAngle = pos;
      }

      public void setLimits(double min, double max){
            this.minAngleToArm = min;
            this.maxAngleToArm = max;
      }

      public double getWristAngleFromFloor(){
            // return targetWristToFloorAngle;
            return wristToDegrees.get(wristMotor.getSelectedSensorPosition());
      }

      public double getWristAngleFromArm(){
            return wristToDegrees.get(wristMotor.getSelectedSensorPosition())-armPosition;
      }

      public boolean isOutOfBounds(){
            if( ! Clamp.bounded(getWristAngleFromArm(), minAngleToArm-5, maxAngleToArm+5)) return true;
            return false;
      }

      public boolean isOnTarget(double tolerance){
            return Clamp.bounded( getWristAngleFromFloor(), targetWristToFloorAngle-tolerance, targetWristToFloorAngle+tolerance);
      }
      
      public void set(Pose pose){
            setTargetAngleFromFloor(pose.wristAngle());
            this.wristPose = pose;
      }

      public void setPower(double pwr){
            this.wristPose = Pose.CUSTOM;
            this.wristPower = pwr;
      }

      public boolean isLimitPressed(){      
            // if(!wristLimit.get()){
            if(wristMotor.getSensorCollection().isFwdLimitSwitchClosed()){ // normally open or close??
                  return true;
            }
            if(wristMotor.getSensorCollection().isRevLimitSwitchClosed()){
                  return true;
            }
            return false;
      }

      public boolean isHomed(){return this.isHomed;}
      public void setHomedReversed(){
            isHomed = true;
            int encoderValue = (int)wristToDegrees.getReverse(armPosition - 90);
            wristMotor.setSelectedSensorPosition(encoderValue, 0, 20);
      }
      public void setHomedForward(){
            isHomed = true;
            int encoderValue = (int)wristToDegrees.getReverse(armPosition + 90);
            wristMotor.setSelectedSensorPosition(encoderValue, 0, 20);
      }


      //Would we even need state enums like this? 
      /* public enum Track{
            ARM_ANGLE, FLOOR_ANGLE
      }
      */
      
      /** NOTE: This update is dependent on other functions, and requires data to function properly. */
      public void update(double currentActualArmPosition){
            this.armPosition = currentActualArmPosition;
            double angleFromFloor = getWristAngleFromFloor();
            double angleFromArm = getWristAngleFromArm();

            // Create a shadow variable in local scope to avoid incorrectly altering our target
            // This is needed as we may have to constrain the target due to dynamic influences,
            // which when changed should result in a change of our actual position
            double targetWristToArmAngle = 0;


            switch(mode){
                  case MANUAL:
                  break;

                  case CLOSEDLOOP:
                        // Figure out the angle needed to be at our target floor angle
                        //targetWristToArmAngle = targetWristToFloorAngle - currentActualArmPosition;

                        wristPower = pidWrist.getOutput(angleFromFloor, targetWristToFloorAngle);
                        SmartDashboard.putNumber("Wrist/Output PID", wristPower);
                        wristPower += kWristFF*Math.cos(Math.toRadians(angleFromFloor));

                        //SmartDashboard.putNumber("FB Function Wrist", FB.fb(targetWristToFloorAngle, angleFromFloor, kWristGain));
                        SmartDashboard.putNumber("Wrist/Output FF", kWristFF*Math.cos(Math.toRadians(angleFromFloor)));

                  break;

                  case HOMING:
                        // NOTE: We do not actually have limit switches for this.
                        // Instead, we would need to trust gravity or initial boot-up position
                        //  if(switchfunction.pressed) {
                        //        wristPower = 0;
                        //  }
                        //  else {
                        //        wristPower = -0.3;
                        //  }
                  break;

                  case DISABLED:
                        //TODO: Disabled should set all power variables to zero
                        wristPower = 0;
                  break;

            }

            // AUTO HOMING CORRECTION
            //if (armPositon straight down ) and (expect wrist to be at switch)
                  //expect wrist to be at switch
                  //if not at switch, add motor power
                  //if at swtich, zero out
            if(targetWristToFloorAngle >= -10 && armPosition < -80){

                  // if!lastIsPressed && isLimitPressed();
                  //double lastIsPRessed = isLimitPressed()
                  if(isLimitPressed()){
                        setHomedForward();
                  }
                  else{
                        if(Robot.isCompbot){
                              wristPower = Clamp.clamp(wristPower,HOME_OUTPUT_POWER_MIN,1);
                        }
                        else{
                              //broken, until switches working
                              wristPower = Clamp.clamp(wristPower,HOME_OUTPUT_POWER_MIN,1);
                        }
                  }
            }
            
            // if(targetWristToFloorAngle >= -10 && armPosition > 70) {

            //       if(isLimitPressed()) {
            //             setHomedReversed();
            //       }
            // }
            
            //TODO Check for physical limits based on arm angles
            //if(wristPower > 0 && angleFromArm > MAX_ANGLE_TO_ARM) wristPower = 0;
            //if(wristPower < 0 && angleFromArm < MIN_ANGLE_TO_ARM) wristPower = 0;

            // wristPower = Clamp.clamp(wristPower, -0.2, 0.2);
            // wristPower *= -1;
            wristMotor.set(ControlMode.PercentOutput, wristPower);

            //SimpleWidget wp = ArmElevator.armavatorTab.add("Wrist Power", wristPower);
            
      }


      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
