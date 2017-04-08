package org.usfirst.frc.team1002.system;

import org.usfirst.frc.team1002.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The drive system.
 */
public class DriveSystem implements Runnable {

	/**
	 * The robot drive.
	 */
	public static RobotDrive robotDrive;
	
	/**
	 * The ports of the drive motors.
	 */
	private enum Channel {
		/**
		 * Front left drive motor port.
		 */
		FRONT_LEFT(8),
		/**
		 * Back left drive motor port.
		 */
		REAR_LEFT(9),
		/**
		 * Front right drive motor port.
		 */
		FRONT_RIGHT(5),
		/**
		 * Back right drive motor port.
		 */
		REAR_RIGHT(2),
		;

		/**
		 * The port stored within each enum element.
		 */
		public int port;

		/**
		 * Create a new channel based on a port number.
		 * @param port The port number
		 */
		Channel(int port) {
			this.port = port;
		}
	}

	/**
	 * Deadband and smooth input values.
	 * 
	 * @param value The value to normalize
	 * @return The normalized value
	 */
	private static double normalize(double value) {
		if (value > 0.99) return 1;
		return Math.abs(value) > 0.1 ? Math.pow(value, 3) : 0;
	}

	/**
	 * Creates the drive system with determined defaults.
	 */
	public DriveSystem() {
		// initialize new robot drive with Victor SPs
		robotDrive = new RobotDrive(
				new VictorSP(Channel.FRONT_LEFT.port), 
				new VictorSP(Channel.REAR_LEFT.port), 
				new VictorSP(Channel.FRONT_RIGHT.port),
				new VictorSP(Channel.REAR_RIGHT.port));

		// invert front left and back left motors
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
	}

	/**
	 * The main drive thread.
	 */
	@Override
	public void run() {
		// don't run this threaded operation if we've been interrupted
		if (!Thread.interrupted()) {
			// main drive branching
			if (Robot.driver.getBumper(Hand.kLeft)) {
				// while left bumper, point turn counter clockwise
				robotDrive.mecanumDrive_Polar(0, 0, -1);
			} else if (Robot.driver.getBumper(Hand.kRight)) {
				// while right bumper, point turn clockwise
				robotDrive.mecanumDrive_Polar(0, 0, 1);
			} else {
				// if no bumper is pressed, use normal driving
				double x = normalize(Robot.driver.getX(Hand.kLeft));
				double y = normalize(Robot.driver.getY(Hand.kLeft));
				// get magnitude of x and y left axes using pythag theorem
				double mag = Math.sqrt(x * x + y * y);
				// get angle between x and y left axes using arctan
				double dir = Math.toDegrees(Math.atan2(x, -y));
				// get right x axis for spin
				double t = normalize(Robot.driver.getX());
				// apply to drive
				robotDrive.mecanumDrive_Polar(mag, dir, t);
			}
		}
	}
}
