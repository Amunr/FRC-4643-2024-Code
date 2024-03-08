package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.motorConstants;

import com.revrobotics.CANSparkMax;

public class climberSubsystem {
     private CANSparkMax leftClimberMotor = new CANSparkMax(motorConstants.kclimberLeftPort, MotorType.kBrushless);
    private CANSparkMax rightClimberMotor = new CANSparkMax(motorConstants.kclimberRightPort, MotorType.kBrushless);

    public void leftClimberControl (int speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control);
        }
    }

    public void rightClimberControl (int speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            rightClimberMotor.set(speed_control);
        }

    }
}
