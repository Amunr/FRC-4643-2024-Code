package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.motorConstants;
public class shooterSubystem {
    private CANSparkMax rightMotor = new CANSparkMax(motorConstants.kshooterRightPort, MotorType.kBrushed);
    private CANSparkMax leftMotor = new CANSparkMax(motorConstants.kshooterLeftPort, MotorType.kBrushed);

    public void StartShooter() {
        rightMotor.set(0.1);
        leftMotor.set(0.1);
    }
    public void stopShooter(){
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }
}
