package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.motorConstants;
import frc.robot.Constants.portConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.RobotContainer;

// LEFT FROM INTAKE
public class climberSubsystem extends SubsystemBase {
     private static CANSparkMax leftClimberMotor = new CANSparkMax(motorConstants.kclimberLeftPort, MotorType.kBrushless);
    private static CANSparkMax rightClimberMotor = new CANSparkMax(motorConstants.kclimberRightPort, MotorType.kBrushless);
        private static RelativeEncoder leftCimberEncoder = leftClimberMotor.getEncoder();
        private static RelativeEncoder rightClimberEncoder = rightClimberMotor.getEncoder();
        public static double leftDistance = 0;
        public static double rightDistance = 0;
    public void getEncoderStatus(){
         leftDistance = leftCimberEncoder.getPosition();
         rightDistance = rightClimberEncoder.getPosition();
    } 
    //Climber maintenance
    public void maintenanceLeftClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control *0.75);
        }
    }
//Climer maintenance
    public void maintenanceRightClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            rightClimberMotor.set(speed_control * 0.75);
        }

    }

     public void leftClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control *0.75);
        }
    }
//Climer maintenance
public void rightClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            rightClimberMotor.set(speed_control * 0.75);
        }

    }

     public void leftCimberEncoder (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            leftClimberMotor.set(speed_control *0.75);
        }
    }
    public void periodic() {
        getEncoderStatus();
        //  double leftControl = RobotContainer.operatorXbox.getLeftY();
        //  leftClimberControl(leftControl);
        //  double rightControl = RobotContainer.operatorXbox.getRightY();
        //  rightClimberControl(rightControl);
         SmartDashboard.putNumber("LeftClimberEncoder", climberSubsystem.leftDistance);
         SmartDashboard.putNumber("RightClimberEncoder", climberSubsystem.rightDistance);
         double maintenanceLeftControl = RobotContainer.operatorXbox.getLeftY();
         maintenanceLeftClimberControl(maintenanceLeftControl);
         double maintenanceRightControl = RobotContainer.operatorXbox.getRightY();
         maintenanceRightClimberControl(maintenanceRightControl);
         //Should not be here(sensors)
    }
}
