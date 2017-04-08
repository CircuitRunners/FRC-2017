package org.usfirst.frc.team1002.robot;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;

public class camSystem {
	//Author:Nathan Pendergrast
	private UsbCamera fwdserver, bckserver;
	private CvSink fwdsink, bcksink, activesink;
	private static double framedelay = 0.05;
	private static int reswidth = 160, resheight = 120;
	static final int FWD = 0;
	static final int BCK = 1;
	int activeCamera = -1;
	Thread camThread;

	void init() {
		camThread = new Thread(new Runnable() {
			public void run() {
				cameraOperation();
			}
		});
		camThread.setName("CameraThread");
		camThread.start();
	}

	void setFeed(int cam) {
		if (activeCamera == cam)
			return;
		if (cam == FWD) {
			activesink = fwdsink;
			activeCamera = FWD;
		} else {
			activesink = bcksink;
			activeCamera = BCK;
			// USED FOR SETTING CAMERA BUTTON/STATE
		}
	}

	protected void cameraOperation() {

		fwdserver = CameraServer.getInstance().startAutomaticCapture("fwd", FWD);
		fwdserver.setBrightness(5);
		fwdserver.setExposureManual(80);
		fwdserver.setResolution(reswidth, resheight);
		fwdsink = CameraServer.getInstance().getVideo(fwdserver);
		bckserver = CameraServer.getInstance().startAutomaticCapture("bck", BCK);
		bckserver.setResolution(reswidth, resheight);
		bcksink = CameraServer.getInstance().getVideo(bckserver);
		CvSource liveFeed = CameraServer.getInstance().putVideo("Livestream", reswidth, resheight);
		Mat nathansMat = new Mat();
		setFeed(FWD);
		// activesink = fwdsink;
		while (!Thread.interrupted()) {

			if (activesink.grabFrame(nathansMat) == 0) {
				liveFeed.notifyError(activesink.getError());
			} else {

				liveFeed.putFrame(nathansMat);
			}
			if (Robot.driver.getRawButton(Xbox.BTN_X)) {
				if (Robot.operator.getRawButton(Xbox.BTN_RIGHT_BUMPER)) {
					if (activeCamera == FWD) {
						setFeed(BCK);
					} else {
						setFeed(FWD);
					}
				}
			} else {
				if (Robot.driver.getRawButton(Xbox.BTN_RSTICK)) {
					if (activeCamera == FWD) {
						setFeed(BCK);
					} else {
						setFeed(FWD);
					}
				}
			}
			Timer.delay(framedelay);
		}
	}
}
