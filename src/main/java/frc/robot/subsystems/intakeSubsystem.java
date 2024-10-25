package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.motorConstants;

public class intakeSubsystem {
    private CANSparkMax intakeMotor = new CANSparkMax(motorConstants.kIntakeMotorPort, MotorType.kBrushless);

    public void StartIntake() {
                SmartDashboard.putString("Intake", "ON");

        intakeMotor.set(1);
    }

    public void manualIntake(){
                        SmartDashboard.putString("Intake", "ON");

        intakeMotor.set(0.5);
    }
    public void stopIntake(){
                        SmartDashboard.putString("Intake", "OFF");

        intakeMotor.stopMotor();
    }
    public void reverseIntake(){
                        SmartDashboard.putString("Intake", "REVERSE");

        intakeMotor.set(-0.5);
    }

}
