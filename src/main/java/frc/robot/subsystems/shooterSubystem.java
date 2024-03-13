package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.motorConstants;
import frc.robot.Constants.shooterConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class shooterSubystem extends SubsystemBase {
    private CANSparkMax shooterMotor = new CANSparkMax(motorConstants.kShooter, MotorType.kBrushless);
    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

    private PIDController shooterPID = new PIDController(
        shooterConstants.kProportoinal, shooterConstants.kIntegral,
            shooterConstants.kDerivative);
    private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
        shooterConstants.kStaticGain, shooterConstants.kVoltage);

    public boolean shooterEnabled = false;

    public void StartShooter() {
        shooterEnabled = true;
    }

    @Override
    public void periodic() {
        if (shooterEnabled) {
            shooterMotor.set(shooterPID.calculate(shooterEncoder.getVelocity(), 4000)
                    + shooterFeedforward.calculate(shooterEncoder.getVelocity()));
        } else {
            shooterMotor.stopMotor();
        }
    }

    public void stopShooter() {
        shooterEnabled = false;
    }

}
