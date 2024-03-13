package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;
import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;

public class climberSubsystem extends SubsystemBase {
     private CANSparkMax leftClimberMotor = new CANSparkMax(motorConstants.kclimberLeftPort, MotorType.kBrushless);
    private CANSparkMax rightClimberMotor = new CANSparkMax(motorConstants.kclimberRightPort, MotorType.kBrushless);
    public void leftClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control);
        }
    }

    public void rightClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            rightClimberMotor.set(speed_control);
        }

    }
    @Override
    public void periodic() {
         double leftControl = RobotContainer.operatorXbox.getLeftY();
         leftClimberControl(leftControl);
         double rightControl = RobotContainer.operatorXbox.getRightY();
         rightClimberControl(rightControl);
    }
}
