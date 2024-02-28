package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;
public class indexerSubystem {
    private CANSparkMax indexMotor = new CANSparkMax(motorConstants.kIndexerMotorPort, MotorType.kBrushless);

    public void startIndexer(){
        indexMotor.set(0.1);
    }
    public void stopIndexer(){
        indexMotor.stopMotor();
    }
    public void reverseIndexer(){
        indexMotor.set(-0.1);
    }
}
