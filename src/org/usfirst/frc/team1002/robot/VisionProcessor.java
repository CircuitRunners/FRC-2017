package org.usfirst.frc.team1002.robot;

import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team1002.iteraters.Iterate;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoProperty;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class VisionProcessor {

}
	/*
	 * Author: Nathan Pendergrast
	 * 
	 * HUGE shoutout to East Cobb Robotics, especially their programmer Jason
	 * Cobb for helping me learn to code for vision tracking and supplying me
	 * with the values for the tape dimensions on the gear peg. This team has no
	 * idea how much help they have given me as a rookie programmer by being
	 * willing to help.
	 * 
	 *
	*
	 * @SuppressWarnings("unused") private static VisionProcessor test;
	 * 
	 * @SuppressWarnings("unused") public static double gyroStartValue; //
	 * values for the microsoft CM_300 webcam public static final String
	 * practicePegUSBport = "0"; public static final String compPegUSBport =
	 * "1"; private static VisionProcessor instance; private static final double
	 * verticalFOVpeg = 34.3; private static final double horizontalFOVpeg =
	 * 61.0; private static final double resolutionYpeg = 120; private static
	 * final double resolutionXpeg = 160; private static final double
	 * degreePerPixelYpeg = verticalFOVpeg / resolutionYpeg; private static
	 * final double degreePerPixelXpeg = horizontalFOVpeg / resolutionXpeg;
	 * private static final double centerImgYpeg = 79.5; private static final
	 * double centerImgXpeg = 59.5;
	 * 
	 * /* private static final double verticalFOVhighgoal = 34.3; private static
	 * final double horizontalFOVhighgoal = 61.0; private static final double
	 * resolutionYhighgoal = 120; private static final double
	 * resolutionXhighgoal = 160; private static final double
	 * degreePerPixelYhighgoal = verticalFOVhighgoal / resolutionYhighgoal;
	 * private static final double degreePerPixelXhighgoal =
	 * horizontalFOVhighgoal / resolutionXhighgoal; private static final double
	 * centerImgYhighgoal = 79.5; private static final double centerImgXhighgoal
	 * = 59.5;
	 

	private static final double pegTapeWidth = 2.0;
	private static final double pegTapeHeight = 5.0;
	private static final double pegTapeGap = 6.25;
	private static final double pegTapeArea = 10.0;
	private static final double pegAspectRatio = pegTapeWidth / pegTapeHeight;
	private static final double heightPegCenter = 7.0;
	private static final double minPegClearance = 30;

	 * @SuppressWarnings("unused") private static final double optimalShot =
	 * 102.2; private static final double optimalShotAngle = 86.33; private
	 * static final double heightGoalCenter = 0.0; private static final double
	 * goalCamFromCenterX = 0.0; private static final double goalCamFromCenterY
	 * = 0.0;
	 *
	 * @SuppressWarnings("unused") private static double
	 * distanceToGoal(doublecalculatedDistance, double t) { double X =
	 * calculatedDistance * Math.sin(Math.toRadians(t)); double Y =
	 * calculatedDistance * Math.cos(Math.toRadians(t)); double ans =
	 * Math.hypot(X + goalCamFromCenterX, Y + goalCamFromCenterY); return ans; }
	 * 
	 * private static double angleToGoal(double calculatedDistance, double t) {
	 * double X = calculatedDistance * Math.sin(Math.toRadians(t)); double Y =
	 * calculatedDistance * Math.cos(Math.toRadians(t)); return
	 * Math.toDegrees(Math.atan((X + goalCamFromCenterX) / (Y +
	 * goalCamFromCenterY))); }
	 *

	@SuppressWarnings("unused")
	private static HashMap<String, VideoProperty> PropertyMap = new HashMap<String, VideoProperty>();

	private static double yawAngleToTargetApproxX(double error) {
		return error * degreePerPixelXpeg;
	}

	private static double yawAngleToTargetApproxY(double error) {
		return error * degreePerPixelYpeg;
	}

	private static final double focalLengthX = resolutionXpeg
			/ (2.0 * Math.tan(Math.toRadians((horizontalFOVpeg / 2.0))));
	private static final double focalLengthY = resolutionYpeg
			/ (2.0 * Math.tan(Math.toRadians((verticalFOVpeg / 2.0))));

	private static double yawAngleToTarget(double error) {
		return Math.toDegrees(Math.atan((error / focalLengthX)));
	}

	private static double pitchAngleToTarget(double error) {
		return Math.toDegrees(Math.atan((error / focalLengthY)));
	}

	private static double distanceToTarget(double error) {
		return heightPegCenter * focalLengthY / error;
	}

	private static double distanceToTargetApprox(double error) {
		return heightPegCenter / Math.tan(Math.toRadians(error * degreePerPixelYpeg));
	}

	private static VideoCapture videoCapture;
	private static Mat BGR, HSV, blur, threshold, clusters, hierarchy;
	private static boolean insideTarget = false;
	private static SerialPort cam;
	private static CvSink cvspeg, cvsboiler;
	private static double captureTime;
	private static UsbCamera pegCam;
	private static UsbCamera boilerCam;
	private static CvSource boilerOutput;
	private static int iteration = 0;
	private static boolean hasBeenEnabled = false;
	private static double currentAngle = 0.0;
	private static double currentDistance = 0.0;
	private static double calculatedAngle = 0.0;
	private static double calculatedDistance = 0.0;
	private static double _angle = 0.0, _distance = 0.0;

	private enum VisionStates {
		Disabled, PegTracking, BoilerTracking;
	}

	private VisionStates visionState = VisionStates.Disabled;
	private static final Scalar
	// Color values, supplied by Jason Cobb
	BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0), RED = new Scalar(0, 0, 255),
			WHITE = new Scalar(255, 255, 255), BLACK = new Scalar(0, 0, 0), LOWER_BOUNDS = new Scalar(65, 119, 78),
			UPPER_BOUNDS = new Scalar(97, 255, 146);
	private static Scalar color = BLACK;
	private static double itercount = 0.0;
	private static double pegDistSum = 0.0;
	private static double pegAngleSum = 0.0;
	private final Iterate iter=new Iterate(){

	// @Override
	public void init(){synchronized(VisionProcessor.this){visionState=VisionStates.Disabled;gyroStartValue=Robot.gyro.getAngle();hasBeenEnabled=false;itercount=pegDistSum=pegAngleSum=0.0;}

	}

	@Override public void exec(){synchronized(VisionProcessor.this){switch(visionState){

	case Disabled:if(Robot.operator.getRawButton(Xbox.BTN_START)){startPegTracking();}break;case PegTracking:if(Robot.operator.getRawButton(Xbox.BTN_B)){stopPegTracking();break;}processPeg();pegAngleSum=getCalculatedPegAngle();pegDistSum=getCalculatedPegDistance();itercount++;break;case BoilerTracking:break;default:break;}

	}

	}

	@Override public void end(){synchronized(VisionProcessor.this){visionState=VisionStates.Disabled;itercount=pegDistSum=pegAngleSum=0.0;

	}

	}

	};

	public Iterate getLoop() {
		return iter;
	}

	private VisionProcessor() {
		System.load("/usr/local/frc/lib/libopencv_java310.so");
		BGR = new Mat();
		HSV = new Mat();
		blur = new Mat();
		threshold = new Mat();
		clusters = new Mat();
		hierarchy = new Mat();
		// test to see if this causes errors
		pegCam = CameraServer.getInstance().startAutomaticCapture();
		boilerCam = CameraServer.getInstance().startAutomaticCapture();
		cvspeg = CameraServer.getInstance().getVideo(pegCam);
		cvsboiler = CameraServer.getInstance().getVideo(boilerCam);
	}

	public static VisionProcessor getInstance() {
		return instance == null ? instance = new VisionProcessor() : instance;
	}

	public synchronized void startPegTracking() {
		synchronized (VisionProcessor.this) {
			visionState = VisionStates.PegTracking;
			cvspeg.setEnabled(true);
			Timer.delay(0.05);
			hasBeenEnabled = true;
			cvspeg.grabFrame(BGR);
			insideTarget = false;
		}
	}

	public synchronized void stopPegTracking() {
		synchronized (VisionProcessor.this) {
			visionState = VisionStates.Disabled;
			itercount = pegDistSum = pegAngleSum = 0.0;
			cvspeg.setEnabled(false);
		}
	}

	private synchronized void processPeg() {
		synchronized (VisionProcessor.this) {
			cvspeg.grabFrame(BGR);
			captureTime = Timer.getFPGATimestamp();
			currentAngle = Robot.gyro.getAngle();
			currentDistance = ((myDrive.frontLeft.getDistance() + myDrive.frontRight.getDistance()
					+ myDrive.backLeft.getDistance() + myDrive.backRight.getDistance()) / 4);
			iteration++;
			ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			if (!BGR.empty() && !insideTarget){
				Imgproc.cvtColor(BGR, HSV, Imgproc.COLOR_BGR2HSV);//Converts BGR frames to HSV frames
				Core.inRange(HSV, LOWER_BOUNDS, UPPER_BOUNDS, threshold);// applies the HSV filter
				Imgproc.findContours(threshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);//Finds countours
				
				double contArea;
				
				if(hierarchy.size().height > 0 && hierarchy.size().width > 0 && !insideTarget){
					Rect rect1 = null, rect2 = null;
					Rect totalRect = null;
					boolean hasFoundARect = false;
					for(int idx = 0;idx >= 0 && !insideTarget; idx = (int) hierarchy.get(0, idx)[0]){
						contArea = Imgproc.contourArea(contours.get(idx));
						
						if(contArea > minPegClearance && !insideTarget){
							MatOfPoint approxf1 = new MatOfPoint();
							MatOfPoint2f mMOP2f1 = new MatOfPoint2f();
							MatOfPoint2f mMOP2f2 = new MatOfPoint2f();
							
							contours.get(idx).convertTo(mMOP2f1, CvType.CV_32FC2);
							Imgproc.approxPolyDP(mMOP2f1, mMOP2f2, 7, true);
							mMOP2f2.convertTo(contours.get(idx), CvType.CV_32S);
							
							mMOP2f2.convertTo(approxf1, CvType.CV_32S);
							
							if(!hasFoundARect){
								rect1 = Imgproc.boundingRect(approxf1);
								Point rectMiddle = new Point(rect1.x+.5*rect1.width, rect1.y+.5*rect1.height);
								hasFoundARect = true;
							}else{
								rect2 = Imgproc.boundingRect(approxf1);
								Point rectMiddle = new Point(rect1.x+.5*rect2.width, rect2.y + .5*rect2.height);
								if(Math.min(rect1.tl().x, rect2.tl().x)== rect1.tl().x){
									totalRect = new Rect(rect1.tl(), rect2.br());
								}
								rectMiddle = new Point(totalRect.x+.5*totalRect.width, totalRect.y+ .5*totalRect.height);
								break;
							}
						}
					}
						try{
								double deltaAngle = gyroStartValue - currentAngle;
								double deltaDistance = ((myDrive.frontLeft.getDistance() + myDrive.frontRight.getDistance() + myDrive.backLeft.getDistance() + myDrive.backRight.getDistance())/4) - currentDistance;
								
					}
				}
			}
		}
	}
}*/