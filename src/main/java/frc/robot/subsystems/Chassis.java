/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ChassisTeleopDrive;


/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

  // /*
  //Motors must be burshless or the robot will explode
  public CANSparkMax motorL0 = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax motorL1 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax motorL2 = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax motorR0 = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax motorR1 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax motorR2 = new CANSparkMax(6, MotorType.kBrushless);
  // */

  public SpeedControllerGroup motorsLeft = new SpeedControllerGroup(motorL0, motorL1, motorL2);
  public SpeedControllerGroup motorsRight = new SpeedControllerGroup(motorR0, motorR1, motorR2);

  //Motor 1 and 4 are the leaders of their sides. 1 for left and 4 for right.
  DifferentialDrive drive = new DifferentialDrive(motorsLeft, motorsRight);

  //Gearbox Shifters
  public Solenoid shifter = new Solenoid(2);
  public Solenoid shifterInverse = new Solenoid(5);

  public Ultrasonic sonarL = new Ultrasonic(RobotMap.UltrasonicLeftPing, RobotMap.UltrasonicLeftEcho);
  public Ultrasonic sonarR = new Ultrasonic(RobotMap.UltrasonicRightPing, RobotMap.UltrasonicRightEcho);

  double arcadeForward = 0;
  double arcadeTurn = 0;

  double tankLeft = 0;
  double tankRight = 0;

  private Mode driveMode = Mode.DRIVER;

  // Use an Enum to select a Differentail Drive type
  public enum Mode{
    DRIVER,
    TANKDRIVE
  }


  // Use an Enum to define pnuematic truth values, so that you get good named values 
  // backed by type checking everywhere.
  public enum Gear{
    HIGH(true,false),
    LOW(false,true);
    private boolean compbot,practicebot;
    Gear(boolean compbot, boolean practicebot){
      this.compbot = compbot;
      this.practicebot = practicebot;
    }
    public boolean bool(){return Robot.isCompbot ? this.compbot : this.practicebot;};
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ChassisTeleopDrive());
  }

  /**
   * initializes the ramprates and sets the slave motors
   */
  public Chassis(){
    // For the CANSparkMax's, we would use the smartCurrentLimit()
    //TODO: Figure out good current limits, and if we should use full linear range or the low cap
    // See http://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANSparkMax.html#setSmartCurrentLimit(int,int,int)
    // motorL.setSmartCurrentLimit​(int stallLimit, int freeLimit, int limitRPM)
    //NOTE: Max power draw for  for Neo Brushless at stall is 105Amps
    //NOTE: Max free-spin speed is 5700 RPM

    //In an attempt to budget power across all motors, this is a safe start point that should
    // minimize brownouts
    //TODO: Make sure we budget appropriately for the number of motors we're running
    //TODO: Examine how much power we're using and see where it needs to all go
    int stallLimit = 180/4;  // /6;  
    int freeLimit = 220/4;  // /6;
    int limitRPM = 6700/3;
    motorL0.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorL1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorL2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR0.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

    double rampRate = 260;
    motorL0.setOpenLoopRampRate(rampRate);
    motorL1.setOpenLoopRampRate(rampRate);
    motorL2.setOpenLoopRampRate(rampRate);
    motorR0.setOpenLoopRampRate(rampRate);
    motorR1.setOpenLoopRampRate(rampRate);
    motorR2.setOpenLoopRampRate(rampRate);
  }

  /** Runs on robot boot after network/SmartDashboard becomes available */
  public void robotInit(){
    shift(Gear.LOW);

    // Setup the ultrasonics
    sonarL.setEnabled(true);
    sonarR.setEnabled(true);
    sonarL.setAutomaticMode(true);
    sonarR.setAutomaticMode(true);
    

    if(Robot.isCompbot){
    }
    else{
    }
  }
  

  public void shift(Gear gear){
    shifter.set(gear.bool());
    shifterInverse.set(!gear.bool());
  }

  public void arcadeDrive(double speed,double turn){
    arcadeForward = speed; 
    arcadeTurn = turn;
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankLeft = leftSpeed;
    tankRight = rightSpeed;
  }

  public void setMode(Mode newMode) {
    driveMode = newMode;
  }

  public void update(){

    SmartDashboard.putNumber("Chassis/Left Ultrasonic", sonarL.getRangeInches());
    SmartDashboard.putNumber("Chassis/Right Ultrasonic", sonarR.getRangeInches());
    SmartDashboard.putString("Chassis/CurrentCommand", getCurrentCommandName());
    SmartDashboard.putNumber("Chassis/arcadeForward", arcadeForward);
    SmartDashboard.putNumber("Chassis/arcadeTurn", arcadeTurn);

    switch(driveMode){
      case DRIVER:
        drive.arcadeDrive(arcadeForward, arcadeTurn,false);
        break;
      case TANKDRIVE:
        drive.tankDrive(tankLeft, tankRight);
        break;
    }
  }
}
