package org.usfirst.frc.team1002.system;

import org.usfirst.frc.team1002.robot.Robot;

//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.VictorSP;

public class DriveSystem implements Runnable {

	public static RobotDrive robotDrive;
	public static Encoder encoder; 

	private enum Channel {
		FRONT_LEFT(8), 
		REAR_LEFT(9), 
		FRONT_RIGHT(5), 
		REAR_RIGHT(2),
		;

		public int port;

		Channel(int port) {
			this.port = port;
		}
	}

	public static final double DIST_PER_PULSE = 0.001;

	private static double smooth(double value) {
		return Math.abs(value) > 0.1 ? Math.pow(value, 3) : 0;
	}

	public DriveSystem() {
		encoder = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);
		encoder.reset();
		encoder.setDistancePerPulse(DIST_PER_PULSE);

		robotDrive = new RobotDrive(
				new VictorSP(Channel.FRONT_LEFT.port), 
				new VictorSP(Channel.REAR_LEFT.port), 
				new VictorSP(Channel.FRONT_RIGHT.port),
				new VictorSP(Channel.REAR_RIGHT.port));

		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
	}

	@Override
	public void run() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		double x = smooth(Robot.driver.getX(Hand.kLeft));
		double y = smooth(Robot.driver.getY(Hand.kLeft));
		double mag = Math.sqrt(x * x + y * y);
		double dir = Math.toDegrees(Math.atan2(x, -y));
		double t = smooth(Robot.driver.getX());

		robotDrive.mecanumDrive_Polar(mag, dir, t);
	}

	/*
	private AnalogInput UltraIn = new AnalogInput(0);

	public double sonarDistance() {
		double voltage = UltraIn.getValue();
		double dist;
		voltage = voltage - 0.03;
		dist = voltage / 0.38582;
		return dist;
	}
	*/
}
