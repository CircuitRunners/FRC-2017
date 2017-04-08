package org.usfirst.frc.team1002.vision;

import edu.wpi.first.wpilibj.CameraServer;

/**
 * System for all camera based activity.
 */
public class CameraSystem {
	/**
	 * Create the camera system.
	 */
	public CameraSystem() {
		// start capture of the first available camera
		CameraServer.getInstance().startAutomaticCapture();
	}
}
