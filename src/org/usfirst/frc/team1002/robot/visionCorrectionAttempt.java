package org.usfirst.frc.team1002.robot;

public class visionCorrectionAttempt {

	private static visionCorrectionAttempt attempt;
	public static double gyroStartValue;
	// values for the microsoft CM_300 webcam
	public static final String practicePegUSBport = "0";
	public static final String compPegUSBport = "1";
	private static VisionProcessor instance;
	private static final double verticalFOVpeg = 34.3;
	private static final double horizontalFOVpeg = 61.0;
	private static final double resolutionYpeg = 120;
	private static final double resolutionXpeg = 160;
	private static final double degreePerPixelYpeg = verticalFOVpeg / resolutionYpeg;
	private static final double degreePerPixelXpeg = horizontalFOVpeg / resolutionXpeg;
	private static final double centerImgYpeg = 79.5;
	private static final double centerImgXpeg = 59.5;
	
	private static final double pegTapeWidth = 2.0;
	private static final double pegTapeHeight = 5.0;
	private static final double pegTapeGap = 6.25;
	private static final double pegTapeArea = 10.0;
	private static final double pegAspectRatio = pegTapeWidth / pegTapeHeight;
	private static final double heightPegCenter = 7.0;
	private static final double minPegClearance = 30;

	
}
