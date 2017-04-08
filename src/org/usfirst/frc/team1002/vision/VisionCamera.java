package org.usfirst.frc.team1002.vision;

import edu.wpi.first.wpilibj.CameraServer;

public class VisionCamera {
	public VisionCamera() {
		CameraServer.getInstance().startAutomaticCapture();
	}
}
