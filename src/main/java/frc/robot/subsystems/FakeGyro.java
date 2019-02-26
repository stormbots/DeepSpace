/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Sendable;

/**
 * Add your docs here.
 */
public class FakeGyro extends GyroBase implements Sendable{
    double angle = 0;

    public FakeGyro set(double angle){this.angle = angle;return this;}

	@Override
	public void calibrate() {
		
	}

	@Override
	public void reset() {
		
	}

	@Override
	public double getAngle() {
		return this.angle;
	}

	@Override
	public double getRate() {
		return 0;
	}    
}
