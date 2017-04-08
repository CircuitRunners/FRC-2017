package org.usfirst.frc.team1002.system;

import org.usfirst.frc.team1002.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The climb system.
 * 
 * It is an axle wrapped in velcro that attaches to the
 * ropes when they descend in order to climb.
 */
public class ClimbSystem implements Runnable {
	
	/**
	 * The VictorSP motor controller used for the climb system.
	 */
	private static VictorSP motor;
	/**
	 * The port for the motor controller.
	 */
	private static final int PORT = 7;

	/**
	 * Creates the climb system with determined Victor SP.
	 */
	public ClimbSystem() {
		motor = new VictorSP(PORT);
	}

	/**
	 * The thread for climbing.
	 */
	@Override
	public void run() {
		// don't run this threaded operation if we've been interrupted
		if (!Thread.interrupted()) {
			// invert squared inputs from the right trigger axis 
			motor.set(-Math.pow(Robot.driver.getTriggerAxis(Hand.kRight), 2));
		}
	}
}
