package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.motorConstants;

public class Intake {
    private CANSparkMax intakeMotor = new CANSparkMax(motorConstants.kIntakeMotorPort, MotorType.kBrushless);

    public void StartIntake() {
        intakeMotor.set(0.1);
    }
    public void endIntake(){
        intakeMotor.set(0);
    }
}
