package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;

public class intakeSubsystem {
    private CANSparkMax intakeMotor = new CANSparkMax(motorConstants.kIntakeMotorPort, MotorType.kBrushless);

    public void StartIntake() {
        intakeMotor.set(1);
    }
    public void stopIntake(){
        intakeMotor.stopMotor();
    }
    public void reverseIntake(){
        intakeMotor.set(-0.5);
    }
}
