/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Hand extends Subsystem {

    public Solenoid leftHand = new Solenoid(2);
    public Solenoid rightHand = new Solenoid(3);
  
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
  
    boolean deployed = false;
  
    public Hand(){
  
      deployed = false;
      /*leftHand.set(deployed);
      rightHand.set(deployed);
      */
    }

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}