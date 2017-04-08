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
	
	SendableChooser<AutoMode> autoChooser;
	
	private ExecutorService teleopThreadPool;
	private ExecutorService autoThreadPool;

	private enum JoystickChannel {
		DRIVER(0), 
		OPERATOR(1),
		;

		public int port;

		JoystickChannel(int port) {
			this.port = port;
		}
	}

	public static XboxController driver = new XboxController(JoystickChannel.DRIVER.port);
	public static XboxController operator = new XboxController(JoystickChannel.OPERATOR.port);
	
	private enum AutoMode {
		LEFT,
		RIGHT,
		MID,
		NONE,
		TRACKING,
		GYRO,
		;
	}

	@Override
	public void robotInit() {
		// Init systems
		drive = new DriveSystem();
		climber = new ClimbSystem();
		camsys = new VisionCamera();
		gyro = new ADXRS450_Gyro();
		
		// Construct auto selection
		autoChooser = new SendableChooser<AutoMode>();
		autoChooser.addDefault("Mid Auto", AutoMode.MID);
		autoChooser.addObject("Left Auto", AutoMode.LEFT);
		autoChooser.addObject("Right Auto", AutoMode.RIGHT);
		autoChooser.addObject("None", AutoMode.NONE);
		autoChooser.addObject("Tracking", AutoMode.TRACKING);
		autoChooser.addObject("Gyro Test", AutoMode.GYRO);
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
		camsys.setTracking(false);
	}

	@Override
	public void teleopPeriodic() {
		teleopThreadPool.execute(drive);
		teleopThreadPool.execute(climber);
	}

	private Runnable autoThread;
	
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
		camsys.setTracking(true);
		AutoMode selectedAuto = autoChooser.getSelected();
		switch (selectedAuto) {
			case RIGHT:
				autoThread = () -> Auto.sidePegAuto(Side.RIGHT, 0.125, 0, 6, 2, -65, 2, 3, 3);
				break;
			case LEFT:
				autoThread = () -> Auto.sidePegAuto(Side.LEFT, 0.125, 0, 6, 2, 65, 2, 3, 3);
				break;
			case MID:
				autoThread = () -> Auto.autoDriveFwd(0.25, 6, 0, 2.25);
				break;
			case TRACKING:
				autoThread = () -> Auto.autoDriveFwdGyroVision(0.25, 6, 0, 2.25);
			case GYRO:
				autoThread = () -> Auto.autoDriveFwdGyro(0.25, 6, 0, 2.25);
			default:
			case NONE:
				break;
		}
		if (autoThread != null) {
			autoThreadPool.execute(autoThread);
		}
	}
	
	private void awaitTerminationAndShutdown(ExecutorService pool) {
		autoThreadPool.shutdownNow();
		try {
			autoThreadPool.awaitTermination(1, TimeUnit.SECONDS);
		} catch (InterruptedException e) {
			autoThreadPool.shutdownNow();
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
}
