package org.usfirst.frc.team1002.system;

import org.usfirst.frc.team1002.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.VictorSP;

public class ClimbSystem implements Runnable {
	private static final int CLIMBCONTROL = 7;
	private static VictorSP motor;

	public ClimbSystem() {
		motor = new VictorSP(CLIMBCONTROL);
	}

	@Override
	public void run() {
		if (!Thread.interrupted()) {
			motor.set(-Math.pow(Robot.driver.getTriggerAxis(Hand.kRight), 2));
		}
	}
}
