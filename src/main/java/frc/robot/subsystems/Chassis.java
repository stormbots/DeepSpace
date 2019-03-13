/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.Clamp;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.PIDController;
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

  CANPIDController pidControllerL = motorL0.getPIDController();
  CANPIDController pidControllerR = motorR0.getPIDController();

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

  private Mode driveMode = Mode.ARCADEDRIVE;

  // Use an Enum to select a Differentail Drive type
  public enum Mode{
    ARCADEDRIVE,
    FOCUSDRIVE,
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

    // PID coefficients
    double kP = 5e-5; 
    double kI = 0;
    double kD = 0; 
    double kIz = 0; 
    double kFF = 0.000156; 
    double kMaxOutput = 1; 
    double kMinOutput = -1;
    double maxRPM = 5700;
    double minVel = 0;
    double allowedErr = 0;

    // Smart Motion Coefficients
    double maxVel = 5700; // rpm
    double maxAcc = 2900;
    
    // set PID coefficients FOR THE LEFT
    pidControllerL.setP(kP);
    pidControllerL.setI(kI);
    pidControllerL.setD(kD);
    pidControllerL.setIZone(kIz);
    pidControllerL.setFF(kFF);
    pidControllerL.setOutputRange(kMinOutput, kMaxOutput);
    
    // set PID coefficients FOR THE RIGHT
    pidControllerR.setP(kP);
    pidControllerR.setI(kI);
    pidControllerR.setD(kD);
    pidControllerR.setIZone(kIz);
    pidControllerR.setFF(kFF);
    pidControllerR.setOutputRange(kMinOutput, kMaxOutput);
    
    int smartMotionSlot = 0;

    pidControllerL.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidControllerL.setSmartMotionMinOutputVelocity(-maxVel, smartMotionSlot);
    pidControllerL.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidControllerL.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    pidControllerR.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidControllerR.setSmartMotionMinOutputVelocity(-maxVel, smartMotionSlot);
    pidControllerR.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidControllerR.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);



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
    int stallLimit = 160/6;  // /6;  
    int freeLimit = 180/6;  // /6;
    int limitRPM = 6700/3;
    motorL0.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorL1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorL2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR0.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

    double rampRate = 0.2;
    motorL0.setOpenLoopRampRate(rampRate);
    motorL1.setOpenLoopRampRate(rampRate);
    motorL2.setOpenLoopRampRate(rampRate);
    motorR0.setOpenLoopRampRate(rampRate);
    motorR1.setOpenLoopRampRate(rampRate);
    motorR2.setOpenLoopRampRate(rampRate);

    motorL0.setIdleMode(IdleMode.kBrake);
    motorL1.setIdleMode(IdleMode.kCoast);
    motorL2.setIdleMode(IdleMode.kCoast);
    motorR0.setIdleMode(IdleMode.kBrake);
    motorR1.setIdleMode(IdleMode.kCoast);
    motorR2.setIdleMode(IdleMode.kCoast);

    motorL1.follow(motorL0);
    motorL2.follow(motorL0);
    motorR1.follow(motorR0);
    motorR2.follow(motorR0);

  }

  public void reset() {
    motorL0.getEncoder().setPosition(0);
    motorL1.getEncoder().setPosition(0);
    motorL2.getEncoder().setPosition(0);
    motorR0.getEncoder().setPosition(0);
    motorR1.getEncoder().setPosition(0);
    motorR2.getEncoder().setPosition(0);
    //TODO SET TARGET TO 0 AS WELL
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

    //pidControllerL.setReference(JoystickValue, ControlType.kSmartVelocity);
    //pidControllerR.setReference(JoystickValue, ControlType.kSmartVelocity);


    switch(driveMode){
      case ARCADEDRIVE:
        //arcadeDriveOverload(arcadeForward, arcadeTurn);
        drive.arcadeDrive(arcadeForward, arcadeTurn*0.8, false);
        break;
      case FOCUSDRIVE:
        drive.curvatureDrive(arcadeForward,arcadeTurn,false);
        break;
      case TANKDRIVE:
        drive.tankDrive(tankLeft, tankRight);
        break;
    }
  }


  private void arcadeDriveOverload(double xSpeed, double zRotation){
    double leftMotorOutput;
    double rightMotorOutput;

    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      } else {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = maxInput;
      } else {
        leftMotorOutput = maxInput;
        rightMotorOutput = xSpeed - zRotation;
      }
    }

    double leftSidePower = leftMotorOutput;
    double rightSidePower = rightMotorOutput * -1;
    double maximumRPM = 5700;

    SmartDashboard.putNumber("Chassis/Left Side Joystick", leftSidePower);
    SmartDashboard.putNumber("Chassis/Left Side Speed", leftSidePower*maximumRPM);

    pidControllerL.setReference(leftSidePower*maximumRPM, ControlType.kSmartVelocity);
    pidControllerR.setReference(rightSidePower*maximumRPM, ControlType.kSmartVelocity);

  }

  public void periodic(){
    SmartDashboard.putNumber("Chassis/Output actual Left", motorL0.getAppliedOutput());
    SmartDashboard.putNumber("Chassis/Output actual Right", motorR0.getAppliedOutput());
  }
}
