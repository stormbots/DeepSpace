/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.PassThroughPower;
import static com.stormbots.Clamp.*;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class PassThrough extends Subsystem {

public static final double GRAB_BALL_POWER = 0.3 ;
public static final double CARGO_STALL_CURRENT = 3;

  private VictorSPX beltMotor = new VictorSPX(6);
  private VictorSPX beltMotorB = new VictorSPX(6);

  double beltPower = 0;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public PassThrough() {
        //Configure Passthrough belt motors
    //TODO: Figure if this is needed: beltMotor.setInverted(false);
    beltMotorB.follow(beltMotor);
    beltMotorB.setInverted(InvertType.OpposeMaster);
    beltMotor.set(ControlMode.PercentOutput, 0);

  }
  
  

  @Override
  public void initDefaultCommand() {
    
    
    // Set the default command for a subsystem here.
    setDefaultCommand(new PassThroughPower(0));
  }
  public void update() {
        beltMotor.set(ControlMode.PercentOutput,beltPower);

  }

  public void setPower(double newPower){
    beltPower = newPower;
  }

   /** True if ball is in passthrough */
   public boolean hasBall() { 
    //TODO Get the current
    // current = beltMotor.getOutputCurrent();// should work? TalonSRX Only?
    double current = Robot.pdp.getCurrent(9);
    return  !bounded(current, -CARGO_STALL_CURRENT, CARGO_STALL_CURRENT);
  }

}
