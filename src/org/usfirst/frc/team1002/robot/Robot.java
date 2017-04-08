package org.usfirst.frc.team1002.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team1002.robot.Auto.Side;
import org.usfirst.frc.team1002.system.ClimbSystem;
import org.usfirst.frc.team1002.system.DriveSystem;
import org.usfirst.frc.team1002.vision.CameraSystem;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The base robot class.
 */
public class Robot extends IterativeRobot {

	/**
	 * The drive system.
	 */
	public static DriveSystem drive;
	/**
	 * The climber.
	 */
	private static ClimbSystem climber;
	/**
	 * The camera.
	 */
	public static CameraSystem camsys;

	
	/**
	 * Input ports.
	 */
	private enum JoystickChannel {
		/**
		 * The main input from the driver.
		 */
		DRIVER(0),
		;

		public int port;

		JoystickChannel(int port) {
			this.port = port;
		}
	}

	/**
	 * The main input for the driver, an Xbox 360 controller.
	 */
	public static XboxController driver;
	
	private static SendableChooser<String> autoChooser;
	
	private static Runnable autoThread;
	public static ExecutorService autoThreadPool;
	public static ExecutorService teleopThreadPool;
	
	/**
	 * Check if this our first time going through any actionable code paths.
	 */
	public static boolean firstRun;

	@Override
	public void robotInit() {
		// Init systems
		drive = new DriveSystem();
		new Auto();
		climber = new ClimbSystem();
		camsys = new CameraSystem();
		
		// Init input
		driver = new XboxController(JoystickChannel.DRIVER.port);
		
		// Prevent from running things in robot actions that happen in initialization
		firstRun = true;
		
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
		// prepare the robot for teleop
		runInit(true);
		// create a new teleop thread pool
		teleopThreadPool = Executors.newCachedThreadPool();
	}

	@Override
	public void teleopPeriodic() {
		teleopThreadPool.execute(drive);
		teleopThreadPool.execute(climber);
	}
	
	@Override
	public void autonomousInit() {
		// prepare the robot for autonomous
		runInit(false);
		// create a new autonomous thread pool
		autoThreadPool = Executors.newCachedThreadPool();
		
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
		// nothing needs to be cleaned up if we haven't run yet
		if (!firstRun) {
			if (teleopThreadPool != null) {
				awaitTerminationAndShutdown(teleopThreadPool);
				teleopThreadPool = null;
			}
			if (autoThreadPool != null) {
				awaitTerminationAndShutdown(autoThreadPool);
				autoThreadPool = null;
			}
		}
	}
	
	@Override
	public void disabledPeriodic() {
		
	}
	
	/**
	 * Sets up the robot for the runnable actions: teleop or autonomous.
	 * 
	 * @param toggle True for teleop, false for autonomous.
	 */
	private static void runInit(boolean toggle) {
		if (!firstRun) {
			ExecutorService oldPool = toggle ? autoThreadPool : teleopThreadPool;
			// remove auto thread pool during teleop
			if (oldPool != null) {
				awaitTerminationAndShutdown(oldPool);
				if (toggle) {
					autoThreadPool = null;
				} else {
					teleopThreadPool = null;
				}
			}
			ExecutorService newPool = toggle ? teleopThreadPool : autoThreadPool;
			// cleanup previous teleop thread pools
			if (newPool != null) {
				awaitTerminationAndShutdown(newPool);
			}
			// reset the gyro from previous runs
			Auto.gyro.reset();
			// reset the encoder from previous runs
			Auto.encoder.reset();
		} else {
			// it's not the first run anymore
			firstRun = false;
		}
	}
	
	/**
	 * Safely shuts the specified thread pool down.
	 * 
	 * Based upon the example code in {@link ExecutorService}'s Javadoc.
	 * 
	 * @param pool The executor service
	 */
	private static void awaitTerminationAndShutdown(final ExecutorService pool) {
		pool.shutdownNow();
		try {
			pool.awaitTermination(1, TimeUnit.SECONDS);
		} catch (InterruptedException e) {
			pool.shutdownNow();
		}
	}
}
