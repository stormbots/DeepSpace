/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.stormbots.Clamp.bounded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.PassThroughPower;
import edu.wpi.first.wpilibj.Preferences;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class PassThrough extends Subsystem {

  public static final double GRAB_BALL_POWER = 0.3 ;
  public static final double LOAD_BALL_POWER = 0.35 ;
  public static final double CARGO_STALL_CURRENT = 3;

  private TalonSRX beltMotor = new TalonSRX(9);

  double beltPower = 0;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public PassThrough() {

    if(Preferences.getInstance().getBoolean("compbot", true)){
      beltMotor.setInverted(true);
    }
    else{
      beltMotor.setInverted(false);
    }
        //Configure Passthrough belt motors
    //TODO: Figure if this is needed: beltMotor.setInverted(false);
    beltMotor.configOpenloopRamp(0.1);
    beltMotor.set(ControlMode.PercentOutput, 0);
  }
  
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PassThroughPower(0));
  }

  public void update() {
    SmartDashboard.putString("PassthroughCommand", getCurrentCommandName());
    beltMotor.set(ControlMode.PercentOutput,-beltPower);

    SmartDashboard.putNumber("PT Current", beltMotor.getOutputCurrent());
    SmartDashboard.putNumber("PT Poutput", beltPower);
    
  }

  public void setPower(double newPower){
    beltPower = newPower;
  }

  /** True if ball is in passthrough */
  public boolean hasBall() { 
    //TODO Get the current
    double current = beltMotor.getOutputCurrent();
    return  !bounded(current, -CARGO_STALL_CURRENT, CARGO_STALL_CURRENT);
  }

}
