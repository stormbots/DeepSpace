/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.stormbots.Clamp.clamp;
import static com.stormbots.closedloop.FB.fb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pogos extends Subsystem {

  // public Solenoid leftPogo = new Solenoid(5);
  public TalonSRX pogo = new TalonSRX(16); // TODO: SET Pogo DEVICE ID

  public DigitalInput onHabCenter = new DigitalInput(RobotMap.PogoFloorSensor); // TODO: SET hab DigitalInput ID

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // public static final double maxVelocity = 0; //fix this
  // public static final double maxAcceleration = 0;
  public static final double RETRACTED = 0;
  // public static double HAB_2 = 4096; //TODO: Find pogo down encoder ticks
  public static double DEPLOY_HAB_3 = 4096; //TODO: Find pogo down encoder ticks
  public double kPogoGain = 0.002;
  public double targetPos = 0;

  public Pogos(){
    System.out.println("Pogo is Initialized");  
    pogo.setSensorPhase(true); //default for talon + encoder combo

    //TODO Pogo initialization? 
    // Since boot encoder == 0 and ideal targetPos == 0, we don't need to really do much
  }

  /** Runs on robot boot after network/SmartDashboard becomes available */
  public void robotInit(){
    if(Robot.isCompbot){
    }
    else{
    }
  }

  public void update(){
    //TODO: Pogos should, as a safety, retract pogos if nothing is using them
    //if(this.getCurrentCommand() == null) targetPos = 0; //might 
    //optionally, could create a PogoPosition DefaultCommand and set the targetPos in the init

    //TODO: we should clampthe targetPos to within the bounds of the system, just in case
    // Note, that since we're being lazy about this and using ticks, we need to be aware that 
    // DEPLOYED might be negative, and so we'd have to check for that as part of our clamp process

    double outputPower = fb(targetPos, pogo.getSelectedSensorPosition(0), targetPos);

    //TODO: Remove pogo safety clamp
    outputPower = clamp(outputPower,-0.05,0.05); 

    pogo.set(ControlMode.PercentOutput, outputPower );

  }

  public void setPosition(double position){
    this.targetPos = position;
  }

  public boolean isFloorDetected(){
    //TODO: Validate switch default vs triggerd state
    return onHabCenter.get() == true;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
