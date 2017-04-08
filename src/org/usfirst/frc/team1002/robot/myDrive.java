package org.usfirst.frc.team1002.robot;

import org.usfirst.frc.team1002.system.RobotSystem;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class myDrive implements RobotSystem {

	private static RobotDrive robotDrive;

	private enum Channel {
		FRONT_LEFT(8), REAR_LEFT(9), FRONT_RIGHT(5), REAR_RIGHT(2),;

		public int port;

		Channel(int port) {
			this.port = port;
		}
	}

	public static final double DIST_PER_PULSE = 0.0006428574 * 1.7;

	final int opScale = 2;
	static Encoder frontLeft = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
	static Encoder backLeft = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
	static Encoder backRight = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
	static Encoder frontRight = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);

	public static double y;
	public static double x;
	public static double t;

	private static double smooth(double value) {
		return Math.abs(value) > 0.1 ? Math.sin(value) : 0;
	}

	public myDrive() {
		frontLeft.reset();
		backLeft.reset();
		backRight.reset();
		frontRight.reset();

		frontLeft.setDistancePerPulse(DIST_PER_PULSE);
		backLeft.setDistancePerPulse(DIST_PER_PULSE);
		backRight.setDistancePerPulse(DIST_PER_PULSE);
		frontRight.setDistancePerPulse(DIST_PER_PULSE);

		robotDrive = new RobotDrive(Channel.FRONT_LEFT.port, Channel.REAR_LEFT.port, Channel.FRONT_RIGHT.port,
				Channel.REAR_RIGHT.port);

		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true);
	}

	@Override
	public void teleop() {
		// Use the joystick X axis for lateral movement, Y axis for forward
		// movement, and Z axis for rotation.
		x = smooth(Robot.driver.getX(Hand.kLeft));
		y = smooth(Robot.driver.getY(Hand.kLeft));
		t = smooth(Robot.driver.getX());

		myDriveCartesian(x, y, t + Robot.degreeChange, 0.0);
		Robot.lastT = t;
		Robot.lastY = y;
		SmartDashboard.putString("DB/String2", Double.toString(Robot.degreeChange));
	}

	public void auto(double x, double y, double t) {
		myDrive.x = x;
		myDrive.y = y;
		myDrive.t = t;

		myDriveCartesian(x, y, t + Robot.degreeChange, 0.0);
		Robot.lastT = t;
		Robot.lastY = y;
	}

	static double prev_x = 0.0;
	static double prev_y = 0.0;
	static double prev_t = 0.0;
	static final double protectedConstant = 20;

	public void myDriveCartesian(double x, double y, double t, double angle) {

		if (Math.abs(x) > 0.25) {
			x = (prev_x + (x - prev_x) / protectedConstant);
		}
		if (Math.abs(y) > 0.25) {
			y = (prev_y + (y - prev_y) / protectedConstant);
		}
		if (Math.abs(t) > 0.25) {
			t = (prev_t + (t - prev_t) / protectedConstant);
		}
		robotDrive.mecanumDrive_Cartesian(x, y, t, angle);
		prev_x = x;
		prev_y = y;
		prev_t = t;
	}

	public RobotDrive getRobotDrive() {
		return robotDrive;
	}

	public static double getDistance() {
		return frontLeft.getDistance();
	}

	private AnalogInput UltraIn = new AnalogInput(0);

	public double sonarDistance() {
		double voltage = UltraIn.getValue();
		double dist;
		voltage = voltage - 0.03;
		dist = voltage / 0.38582;
		double distt = 0.254;
		return dist;
	}
}
