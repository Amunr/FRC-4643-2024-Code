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
    public void leftClimberControl (double speed_control) {
        if(speed_control < 0.1  & speed_control >-0.1 ){
            leftClimberMotor.stopMotor();
        } else { 
            if(-speed_control > 0.1 & leftDistance < 250){
                leftClimberMotor.set(-speed_control * 0.75);
            } else if(-speed_control < -0.1 & leftDistance > 0){
            leftClimberMotor.set(-speed_control *0.75);
            } 
    }
    }
//RIGHT FROM INTAKE
    public void rightClimberControl (double speed_control) {
        if(speed_control < 0.1  && speed_control >-0.1 ){
            rightClimberMotor.stopMotor();
        } else { 
            if(-speed_control > 0.1 && rightDistance < 250){
                rightClimberMotor.set(-speed_control * 0.75);
            } else if(-speed_control < -0.1 && rightDistance > 0){
            rightClimberMotor.set(-speed_control *0.75);
            } 
        }
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
            rightClimberMotor.set(-speed_control * 0.75);
        }

    }
    public void periodic() {
        getEncoderStatus();
         double leftControl = RobotContainer.operatorXbox.getLeftY();
         leftClimberControl(leftControl);
         double rightControl = RobotContainer.operatorXbox.getLeftY();
         rightClimberControl(rightControl);
         SmartDashboard.putNumber("LeftClimberEncoder", climberSubsystem.leftDistance);
         SmartDashboard.putNumber("RightClimberEncoder", climberSubsystem.rightDistance);
         double maintenanceLeftControl = RobotContainer.maintenanceController.getLeftY();
         maintenanceLeftClimberControl(maintenanceLeftControl);
         double maintenanceRightControl = RobotContainer.maintenanceController.getRightY();
         maintenanceRightClimberControl(maintenanceRightControl);
         //Should not be here(sensors)
    }
}
