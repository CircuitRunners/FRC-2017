package org.usfirst.frc.team1002.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.usfirst.frc.team1002.system.RobotSystem;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The base robot class.
 * 
 * @author Nathan Pendergrast, Nigel Nderi
 */
public class Robot extends IterativeRobot {
	static double lastY;
	public static ADXRS450_Gyro gyro;
	public static double lastT;
	public static double degreeChange;
	
	public static myDrive mechdrive;
	private static climbingMech climber;
	private static shootingMech shooter;
	public static camSystemV camsys = new camSystemV();
	
	SendableChooser<String> autoChooser;
	
	// intakeSystem intake = new intakeSystem();
	static motorBallDump ballDump;

	public enum JoystickChannel {
		DRIVER(0), OPERATOR(1),;

		public int port;

		JoystickChannel(int port) {
			this.port = port;
		}
	}

	public static XboxController driver = new XboxController(JoystickChannel.DRIVER.port);
	public static XboxController operator = new XboxController(JoystickChannel.OPERATOR.port);


	@Override
	public void robotInit() {
		mechdrive = new myDrive();
		climber = new climbingMech();
		autoChooser = new SendableChooser<String>();
		autoChooser.addObject("Left Auto", "left");
		autoChooser.addObject("Right Auto", "right");
		autoChooser.addDefault("Mid Auto", "mid");
		autoChooser.addObject("None", "none");
		autoChooser.addObject("Tracking", "tracking");
		autoChooser.addObject("gyroTEST", "gyro");
		SmartDashboard.putData("Auto Chooser", autoChooser);
		shooter = new shootingMech();
		gyro = new ADXRS450_Gyro();
		ballDump = new motorBallDump();
		// intake.init();
		// operatorMode();
	}

	private ExecutorService threadPool = Executors.newCachedThreadPool();
	private RobotSystemThread driveThread;

	@Override
	public void teleopInit() {
		camsys.setTracking(false);
		driveThread = new RobotSystemThread(mechdrive);
	}

	@Override
	public void teleopPeriodic() {
		// intake.teleOp();
		threadPool.execute(driveThread);
		climber.teleOp();
		shooter.teleOp();
		// servoBallDump.teleOp();
	}

	public class RobotSystemThread implements Runnable {

		private RobotSystem system;

		public RobotSystemThread(RobotSystem system) {
			this.system = system;
		}

		@Override
		public void run() {
			system.teleop();
		}
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void autonomousInit() {
		gyro.reset();
		camsys.setTracking(true);
		String selectedAuto = autoChooser.getSelected();
		switch (selectedAuto) {
		case "right":
			auto.sidePegAuto("right", 0.125, 6, 2, -65, 2, 3, 3);
			break;
		case "left":
			auto.sidePegAuto("left", 0.125, 6, 2, 65, 2, 3, 3);
			break;
		case "mid":
			auto.autoDriveFwd(0.25, 6, 2.25);
			break;
		case "none":
			break;
		case "default":
			auto.midPegAuto(0.25, 3, 3);
		case "tracking":
			auto.autoDriveFwdGyroVision(0.25, 6, 2.25);
		case "gyro":
			auto.autoDriveFwdGyro(0.25, 6, 2.25);
		}
		//camsys.setTracking(false);
	}
}
