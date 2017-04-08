package org.usfirst.frc.team1002.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team1002.robot.Auto.Side;
import org.usfirst.frc.team1002.system.ClimbSystem;
import org.usfirst.frc.team1002.system.DriveSystem;
import org.usfirst.frc.team1002.vision.VisionCamera;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The base robot class.
 * 
 * @author Nathan Pendergrast, Nigel Nderi
 */
public class Robot extends IterativeRobot {

	public static DriveSystem drive;
	private static ClimbSystem climber;
	public static VisionCamera camsys;
	public static ADXRS450_Gyro gyro;
	
	private static SendableChooser<String> autoChooser;
	
	private static Runnable autoThread;
	public static ExecutorService autoThreadPool;
	public static ExecutorService teleopThreadPool;

	private enum JoystickChannel {
		DRIVER(0),
		;

		public int port;

		JoystickChannel(int port) {
			this.port = port;
		}
	}

	public static XboxController driver = new XboxController(JoystickChannel.DRIVER.port);

	@Override
	public void robotInit() {
		// Init systems
		drive = new DriveSystem();
		climber = new ClimbSystem();
		camsys = new VisionCamera();
		gyro = new ADXRS450_Gyro();
		
		// Construct auto selection
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Mid Auto", "MID");
		autoChooser.addObject("Left Auto", "LEFT");
		autoChooser.addObject("Right Auto", "RIGHT");
		autoChooser.addObject("None", "NONE");
		autoChooser.addObject("Tracking", "TRACKING");
		autoChooser.addObject("Gyro Test", "GYRO");
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}
	
	@Override
	public void robotPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		if (autoThreadPool != null) {
			awaitTerminationAndShutdown(autoThreadPool);
			autoThreadPool = null;
		}
		if (teleopThreadPool != null) {
			awaitTerminationAndShutdown(teleopThreadPool);
		}
		teleopThreadPool = Executors.newCachedThreadPool();
		gyro.reset();
		DriveSystem.encoder.reset();
	}

	@Override
	public void teleopPeriodic() {
		teleopThreadPool.execute(drive);
		teleopThreadPool.execute(climber);
	}
	
	@Override
	public void autonomousInit() {
		if (teleopThreadPool != null) {
			awaitTerminationAndShutdown(teleopThreadPool);
			teleopThreadPool = null;
		}
		if (autoThreadPool != null) {
			awaitTerminationAndShutdown(autoThreadPool);
		}
		autoThreadPool = Executors.newCachedThreadPool();
		gyro.reset();
		DriveSystem.encoder.reset();
		String selectedAuto = autoChooser.getSelected();
		switch (selectedAuto) {
			case "RIGHT":
				autoThread = () -> Auto.sidePegAuto(Side.RIGHT, 0.25, 0, 6, 2, -65, 2, 3, 3);
				break;
			case "LEFT":
				autoThread = () -> Auto.sidePegAuto(Side.LEFT, 0.25, 0, 6, 2, 65, 2, 3, 3);
				break;
			case "MID":
				autoThread = () -> Auto.autoDriveFwd(0.25, 0, 6, 2.25);
				break;
			case "TRACKING":
				autoThread = () -> Auto.autoDriveFwdGyroVision(0.25, 0, 6, 2.25);
			case "GYRO":
				autoThread = () -> Auto.autoDriveFwdGyro(0.25, 0, 6, 2.25);
			case "NONE":
				break;
		}
		if (autoThread != null) {
			autoThreadPool.execute(autoThread);
		}
	}

	@Override
	public void autonomousPeriodic() {
		
	}
	
	@Override
	public void disabledInit() {
		if (teleopThreadPool != null) {
			awaitTerminationAndShutdown(teleopThreadPool);
			teleopThreadPool = null;
		}
		if (autoThreadPool != null) {
			awaitTerminationAndShutdown(autoThreadPool);
			autoThreadPool = null;
		}
	}
	
	@Override
	public void disabledPeriodic() {
		
	}
	
	private static void awaitTerminationAndShutdown(final ExecutorService pool) {
		pool.shutdownNow();
		try {
			pool.awaitTermination(1, TimeUnit.SECONDS);
		} catch (InterruptedException e) {
			pool.shutdownNow();
		}
	}
}
