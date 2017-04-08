package org.usfirst.frc.team1002.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class intakeSystem {
	//Author:Nigel Nderi
	private static final int INTAKECONTROL = 6;
	SpeedController motorIntake;

	void init() {
		motorIntake= new Talon(INTAKECONTROL);

	}

	void teleOp() {
		boolean valueR = Robot.driver.getRawButton(Xbox.BTN_RIGHT_BUMPER);
		boolean valueL = Robot.driver.getRawButton(Xbox.BTN_LEFT_BUMPER);
		if(valueR){
			motorIntake.set(1);
		}else if(valueL){
			motorIntake.set(-1);
		}else{
			motorIntake.set(0);
		}
	}
}
