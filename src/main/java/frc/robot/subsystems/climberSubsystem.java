package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;
import frc.robot.Constants.portConstants;

import com.revrobotics.CANSparkMax;

import frc.robot.RobotContainer;
// LEFT FROM INTAKE
public class climberSubsystem extends SubsystemBase {
     private CANSparkMax leftClimberMotor = new CANSparkMax(motorConstants.kclimberLeftPort, MotorType.kBrushless);
    private CANSparkMax rightClimberMotor = new CANSparkMax(motorConstants.kclimberRightPort, MotorType.kBrushless);
    public void leftClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control *0.75);
        }
    }
//RIGHT FROM INTAKE
    public void rightClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            rightClimberMotor.set(-speed_control * 0.75);
        }

    }
    @Override
    public void periodic() {
         double leftControl = RobotContainer.operatorXbox.getLeftY();
         leftClimberControl(leftControl);
         double rightControl = RobotContainer.operatorXbox.getRightY();
         rightClimberControl(rightControl);

         //Should not be here(sensors)
    }
}
