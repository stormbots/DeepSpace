/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.*;

/**
 * Add your docs here.
 */
public class ArmElevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	/*public TalonSRX elevMotor = new TalonSRX(10);
	public TalonSRX armMotor = new TalonSRX(12);
      DigitalInput elevLimit = new DigitalInput(0);
      DigitalInput armLimit = new DigitalInput(1);
      */
      
      /*public Arm arm = new Arm();
      public Elevator elevator = new Elevator();
      */

	public ArmElevator() {

	}

	public enum Mode {
		MANUAL, CLOSEDLOOP, HOMING, SEPARATE, DISABLED
										
      }

	private Mode mode = Mode.SEPARATE;

	public void setMode(Mode newMode) {
            mode = newMode;
            Robot.elev.setMode(mode);
            Robot.arm.setMode(mode);
            
            //change both elevator and arm to correct mode
      }


      public Mode getMode(){
            return mode;
      }

      void update(){

            switch(mode){
                  case MANUAL:
                        //elevator.setMode(ElevMode.MANUAL);
                        //arm.setMode(ArmMode.MANUAL);

                  break;

                  case CLOSEDLOOP:
                        

                  break;

                  case HOMING:
                        

                  break;

                  case SEPARATE:

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
