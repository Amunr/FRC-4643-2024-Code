package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.motorConstants;
import frc.robot.Constants.shooterConstants;
public class shooterSubystem {
    private CANSparkMax shooterMotor = new CANSparkMax(motorConstants.kShooter, MotorType.kBrushless);
    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    private PIDController shooterPID =  new PIDController(shooterConstants.kProportoinal, shooterConstants.kIntegral, shooterConstants.kDerivative);

    public void StartShooter() {
     shooterMotor.set(shooterPID.calculate(shooterEncoder.getCountsPerRevolution(), 12000));
    }
    public void stopShooter(){
        shooterMotor.stopMotor();
    }
}
