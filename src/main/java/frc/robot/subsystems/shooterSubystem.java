package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.motorConstants;
public class shooterSubystem {
    private CANSparkMax shooterMotor = new CANSparkMax(motorConstants.kShooter, MotorType.kBrushless);

    public void StartShooter() {
        shooterMotor.set(0.1);
    }
    public void stopShooter(){
        shooterMotor.stopMotor();
    }
}
