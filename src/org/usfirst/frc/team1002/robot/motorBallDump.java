package org.usfirst.frc.team1002.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

public class motorBallDump {
	//Author:Nathan Pendergrast (that's why it does not work :P)
	private static final int BALLDUMP = 5;
	SpeedController motor;

	public motorBallDump(){
		/*motor = new Talon(BALLDUMP);
		DigitalInput limit = new DigitalInput(0);
		if (Robot.driver.getRawButton(Xbox.BTN_B)){
			if (limit.get() == true){
				motor.set(0);
				Timer.delay(3);
				motor.set(-1);
			}
		}
*/
	}
}
