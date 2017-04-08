package org.usfirst.frc.team1002.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class climbingMech {
	// Author:Nathan Pendergrast
	private static final int CLIMBCONTROL = 7;
	SpeedController motor;

	public climbingMech() {
		motor = new Talon(CLIMBCONTROL);
	}

	void teleOp() {
		motor.set(-1 * (Math.pow(Robot.driver.getRawAxis(Xbox.AXIS_RTRIGGER),2)));
	}
}
