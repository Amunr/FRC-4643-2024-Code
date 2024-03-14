package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;
public class indexerSubystem {
    private CANSparkMax leftIndexerMotor = new CANSparkMax(motorConstants.kIndexerLeftMotorPort, MotorType.kBrushless);
    private CANSparkMax rightIndexerMotor = new CANSparkMax(motorConstants.kIndexerRightMotorPort, MotorType.kBrushless);

    public void startIndexer(){
        leftIndexerMotor.set(-.75);
        rightIndexerMotor.set(.75);
    }
    public void stopIndexer(){
        leftIndexerMotor.stopMotor();
                rightIndexerMotor.stopMotor();

    }
    public void reverseIndexer(){
        leftIndexerMotor.set(0.1);
        rightIndexerMotor.set(-0.1);

    }
}
