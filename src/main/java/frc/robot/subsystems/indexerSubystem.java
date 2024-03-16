package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;
public class indexerSubystem {
    private CANSparkMax leftIndexerMotor = new CANSparkMax(motorConstants.kIndexerLeftMotorPort, MotorType.kBrushless);
    private CANSparkMax rightIndexerMotor = new CANSparkMax(motorConstants.kIndexerRightMotorPort, MotorType.kBrushless);

    public void startIndexer(){
                        SmartDashboard.putString("Indexer", "ON");

        leftIndexerMotor.set(-.75);
        rightIndexerMotor.set(.75);
    }
    public void stopIndexer(){
                                SmartDashboard.putString("Indexer", "OFF");

        leftIndexerMotor.stopMotor();
                rightIndexerMotor.stopMotor();

    }
    public void reverseIndexer(){
                                SmartDashboard.putString("Indexer", "REVERSE");

        leftIndexerMotor.set(0.1);
        rightIndexerMotor.set(-0.1);

    }
    public void manualIndexer(){
        leftIndexerMotor.set(-.1);
        rightIndexerMotor.set(0.1);
    }
}
