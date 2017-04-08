package org.usfirst.frc.team1002.vision;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionCamera {
	private UsbCamera fwdserver, bckserver;
	private CvSink fwdsink, bcksink, activesink;
	private static double framedelay = 0.05;
	private static int reswidth = 160, resheight = 120;
	public static final int FWD = 0;
	public static final int BCK = 1;
	private boolean trackingOn = false;
	private static ArrayList<MatOfPoint> contours;
	static final int MAX_SIZE = 20;

	int activeCamera = -1;
	double internal_xAngle = 0.0, internal_zAngle = 0.0;
	public double external_xAngle = 0, external_zAngle = 0;
	boolean internal_TargetFound = false;
	Object trackInfoLock = new Object();

	public void setTracking(boolean tState) {
		trackingOn = tState;
		if (tState && !trackingOn) {
			setFeed(FWD);
		}
		SmartDashboard.putString("tstate", tState ? "ON" : "OFF");
		if (!tState) {
			synchronized (trackInfoLock) {
				internal_TargetFound = false;
			}
		}
	}

	public void setFeed(int cam) {
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

	public boolean getTargetPlacement() {
		boolean temp;
		synchronized (trackInfoLock) {
			external_xAngle = internal_xAngle;
			external_zAngle = internal_zAngle;
			temp = internal_TargetFound;
		}
		return temp;
	}

	public void cameraOperation() {
		GripPipeline gripProcessor = new GripPipeline();
		boolean debugMode = true;
		boolean targetFound = false;
		int numContours = 0;
		int xCenter = 0;
		double xOffset = 0.0, zOffset = 0.0;
		double xAngle = 0.0, zAngle = 0.0;
		int zCenter = 0;
		long pStart = 0, startTime = 0;
		MatOfPoint contour;
		Rect bb;
		fwdserver = CameraServer.getInstance().startAutomaticCapture("fwd", FWD);
		fwdserver.setResolution(reswidth, resheight);
		fwdsink = CameraServer.getInstance().getVideo(fwdserver);
		bckserver = CameraServer.getInstance().startAutomaticCapture("bck", BCK);
		bckserver.setResolution(reswidth, resheight);
		bcksink = CameraServer.getInstance().getVideo(bckserver);
		CvSource liveFeed = CameraServer.getInstance().putVideo("Livestream", reswidth, resheight);
		Mat nathansMat = new Mat();
		startTime = Utility.getFPGATime();
		setFeed(FWD);
		// activesink = fwdsink;
		while (!Thread.interrupted()) {

			if (activesink.grabFrame(nathansMat) == 0) {
				liveFeed.notifyError(activesink.getError());
			} else {
				if (trackingOn) {
					targetFound = false;
					if (debugMode) {
						pStart = Utility.getFPGATime();
						SmartDashboard.putString("TrackStatus", "StartProcess");
					}
					gripProcessor.process(nathansMat);
					if (debugMode) {
						SmartDashboard.putNumber("ProcessTime", Utility.getFPGATime() - pStart);
						pStart = Utility.getFPGATime();
						SmartDashboard.putString("TrackStatus", "GetContours");
					}
					contours = gripProcessor.findContoursOutput(); // get the
					if (debugMode) {
						SmartDashboard.putNumber("GetContoursTime", Utility.getFPGATime() - pStart);
						SmartDashboard.putString("TrackStatus", "ProcessContours");
						SmartDashboard.putNumber("NumContours", contours.size());
					}
					// results
					/*
					 * Process the filtered contours, and figure out the Angles,
					 * etc.
					 */
					numContours = contours.size();
					if (numContours > 2)
						numContours = 2;
					xCenter = zCenter = 0;
					if (numContours > 0) {
						for (int indx = 0; indx < numContours; indx++) {
							contour = contours.get(indx);
							bb = Imgproc.boundingRect(contour);
							xCenter += bb.x + (bb.width / 2);
							zCenter += bb.y + (bb.height / 2);
						}
						xCenter = xCenter / numContours;
						zCenter = zCenter / numContours;
						xOffset = ((double) xCenter - reswidth / 2) / reswidth;
						zOffset = ((double) zCenter - resheight / 2) / resheight;
						xAngle = xOffset * 30; // 60 degrees full FOV
						zAngle = zOffset * 17;// 34 degrees full FOV
						targetFound = true;

						if (debugMode) {
							SmartDashboard.putNumber("xOffset", xOffset);
							SmartDashboard.putNumber("zOffset", zOffset);
							SmartDashboard.putNumber("xCenter", xCenter);
							SmartDashboard.putNumber("zCenter", zCenter);
							SmartDashboard.putNumber("xAngle", xAngle);
							SmartDashboard.putNumber("zAngle", zAngle);
						}
						// Draw a circle where we think the center is
						Imgproc.circle(nathansMat, new Point(xCenter, zCenter), 15, new Scalar(235, 55, 15), 2);
					}
					synchronized (trackInfoLock) {
						internal_xAngle = xAngle;
						internal_zAngle = zAngle;
						internal_TargetFound = targetFound;
					}
				}
				if (debugMode) {
					if (debugMode)
						SmartDashboard.putNumber("ProcessTime", 0.0);
					if (debugMode)
						SmartDashboard.putNumber("GetContoursTime", 0.0);
				}
				liveFeed.putFrame(nathansMat);
				if (debugMode)
					SmartDashboard.putNumber("CameraLoop", Utility.getFPGATime() - startTime);
			}
			/*
			if (Robot.driver.getXButton()) {
				if (Robot.driver.getBumper(Hand.kRight)) {
					if (activeCamera == FWD) {
						setFeed(BCK);
					} else {
						setFeed(FWD);
					}
				}
			} else {
				if (Robot.driver.getStickButton(Hand.kRight)) {
					if (activeCamera == FWD) {
						setFeed(BCK);
					} else {
						setFeed(FWD);
					}
				}
			}
			*/
			Timer.delay(framedelay);
		}
	}
}
