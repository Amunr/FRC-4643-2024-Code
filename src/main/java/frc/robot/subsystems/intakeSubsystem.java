package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;

public class intakeSubsystem {
    private CANSparkMax intakeMotor = new CANSparkMax(motorConstants.kIntakeMotorPort, MotorType.kBrushless);

    public void StartIntake() {
        intakeMotor.set(0.1);
    }
    public void endIntake(){
        intakeMotor.set(0);
    }
}
