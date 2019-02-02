/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
      // Put methods for controlling this subsystem
      // here. Call these from Commands.
      public TalonSRX armMotor = new TalonSRX(12);
      public DigitalInput armLimit = new DigitalInput(0);

      double maxPos = 5000;
      double minPos = 0;
      double armPos = 0;
      double currentPos = 0;
      double fbGain;
      double voltageRampRate;
      double armVel = 0;

      public Arm() {
            reset();
            voltageRampRate = 0.0;
      }

      public void reset() {

      }


      @Override
      public void initDefaultCommand() {
            // Set the default command for a subsystem here.
            // setDefaultCommand(new MySpecialCommand());
      }
}
