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
import com.stormbots.closedloop.FB;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class ArmElevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public TalonSRX elevMotor = new TalonSRX(10);
	public TalonSRX armMotor = new TalonSRX(12);
      DigitalInput elevLimit = new DigitalInput(0);
      DigitalInput armLimit = new DigitalInput(1);

	double eMaxPos = 10000; // placeholder
	double eMinPos = 0;
	double aMaxPos = 5000; // placeholder
	double aMinPos = 0;
	double armPos = 0;
	double elevatorPos = 0;
	double armCurrentPos = 0;
	double elevCurrentPos = 0;
	// add more positions for lvl 1,2,3 of cargo and hatches
	double elevatorFB;
	double armFB;
	double armVel = 0;
	double elevatorVel = 0;

	public ArmElevator() {

	}

	public enum Mode {
		MANUAL, CLOSEDLOOP, HOMING, DISABLED //Manual Velocity may not be necessary or wanted
										
      }

	private Mode mode = Mode.MANUAL;

	public void setMode(Mode newMode) {
		mode = newMode;
      }
      public Mode getMode(){
            return mode;
      }

      void update(){

            switch(mode){
                  case MANUAL:

                  break;

                  case CLOSEDLOOP:
                        
                  break;

                  case HOMING:

                  break;

                  case DISABLED:

                  break;
            }

      }


	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
