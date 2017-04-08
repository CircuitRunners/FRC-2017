package org.usfirst.frc.team1002.robot;

//import edu.wpi.first.wpilibj.TalonSRX;
import com.ctre.CANTalon;
public class shootingMech {

	private static final int SHOOTERCONTROL = 2;
	CANTalon motor;

	void init() {
		motor = new CANTalon(SHOOTERCONTROL);
	}

	void teleOp() {
		if (Robot.operator.getRawAxis(Xbox.AXIS_RTRIGGER) > 0) {
			motor.set(1);
		}
	}
}
