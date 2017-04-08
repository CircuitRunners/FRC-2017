package org.usfirst.frc.team1002.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class auto {
	public static void autoDriveFwd(double speed, double time, double targetDistance) {
		double startingAngle = Robot.gyro.getAngle();
		
		Encoder enc = myDrive.frontRight;
		enc.reset();
		enc.setDistancePerPulse(myDrive.DIST_PER_PULSE);
		double t_0 = Timer.getFPGATimestamp() + time;
		double distance;
		while (Timer.getFPGATimestamp() < t_0 /* &&  myDrive.backRight.getDistance() < targetDistance */) {
			distance = enc.getDistance();
			SmartDashboard.putNumber("Encoder", distance);
			if (distance >= targetDistance) break;
			Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(speed, 0, 0);
			Timer.delay(0.005);
		}
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0);
	}

	public static void autoDriveFwdGyro(double speed, double time, double targetDistance) {
		double startingAngle = Robot.gyro.getAngle();
		
		Encoder enc = myDrive.frontRight;
		enc.reset();
		enc.setDistancePerPulse(myDrive.DIST_PER_PULSE);
		double t_0 = Timer.getFPGATimestamp() + time;
		double distance;
		while (Timer.getFPGATimestamp() < t_0 /* &&  myDrive.backRight.getDistance() < targetDistance */) {
			distance = enc.getDistance();
			SmartDashboard.putNumber("Encoder", distance);
			if (distance >= targetDistance) break;
			Robot.mechdrive.getRobotDrive().mecanumDrive_Cartesian(0, speed, 0, Robot.gyro.getAngle());
			Timer.delay(0.005);
		}
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0);
	}
	
	public static void autoDriveFwdGyroVision(double speed, double time, double targetDistance) {
		double startingAngle = Robot.gyro.getAngle();
		
		Encoder enc = myDrive.frontRight;
		enc.reset();
		enc.setDistancePerPulse(myDrive.DIST_PER_PULSE);
		double t_0 = Timer.getFPGATimestamp() + time;
		double distance;
		while (Timer.getFPGATimestamp() < t_0 /* &&  myDrive.backRight.getDistance() < targetDistance */) {
			distance = enc.getDistance();
			SmartDashboard.putNumber("Encoder", distance);
			if (distance >= targetDistance) break;
			if (Robot.camsys.getTargetPlacement()) {
				Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(speed, 0, Math.sin(Math.toDegrees(Robot.camsys.external_xAngle)));
			} else {
				Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(speed, 0, Math.sin(Math.toDegrees(startingAngle - Robot.gyro.getAngle())));
			}
			Timer.delay(0.005);
		}
		
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0);
	}
	public static void autoDriveFwdGyroVisionNew(double speed, double time, double targetDistance) {
		double startingAngle = Robot.gyro.getAngle();
		
		Encoder enc = myDrive.frontRight;
		enc.reset();
		enc.setDistancePerPulse(myDrive.DIST_PER_PULSE);
		double t_0 = Timer.getFPGATimestamp() + time;
		double distance;
		while (Timer.getFPGATimestamp() < t_0 /* && myDrive.backRight.getDistance() < targetDistance */) {
			distance = enc.getDistance();
			SmartDashboard.putNumber("Encoder", distance);
			if (distance >= targetDistance) break;
			if (Robot.camsys.getTargetPlacement()) {
				Robot.mechdrive.auto(speed, 0, Math.sin(Math.toDegrees(Robot.camsys.external_xAngle)));
			} else {
				Robot.mechdrive.auto(speed, 0, Math.sin(Math.toDegrees(startingAngle - Robot.gyro.getAngle())));
			}
			Timer.delay(0.005);
		}
		
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0);
	}
	
	private static boolean angleIsPositive(double angle){
		return Robot.gyro.getAngle() > angle;
	}
	
	private static void turn(double angle, double time, String side){
		double t = Timer.getFPGATimestamp();
		switch(side){
			case "left":
				while(Timer.getFPGATimestamp() < t + time && Robot.gyro.getAngle() < angle){
					Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0.5);
				}
			case "right":
				while(Timer.getFPGATimestamp() < t + time && Robot.gyro.getAngle() > angle){
					Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,-0.5);
				}
			default:
				break;
		}
		
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0,0,0);
	}
	
	public static void sidePegAuto(String side, double speed, double forwardTime1, double forwardDistance1,double angle, double turnTime, double forwardTime2, double forwardDistance2){
		autoDriveFwd(speed, forwardTime1, forwardDistance1);
		turn(angle, turnTime, side);
		autoDriveFwd(speed, forwardTime2, forwardDistance2);
		Robot.mechdrive.getRobotDrive().mecanumDrive_Polar(0, 0, 0);
	}
	
	public static void midPegAuto(double speed, double time, double distance){
		autoDriveFwd(speed, time, distance);
	}
	/*
	 * public static autoCorrect(){ double desiredAngle; double correctedAngle;
	 * double actualAngle = Robot.gyro.getInput(); if(){ desiredAngle = 0; }
	 * else{ desiredAngle = myDrive.prev_t; } correctedAngle = desiredAngle +
	 * (desiredAngle - actualAngle); return correctedAngle; }
	 */
}
