package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.portConstants;
import frc.robot.subsystems.shooterSubystem;
public class Sensors {
    public static AnalogInput intakeBeamBreak = new AnalogInput(portConstants.intakeBeamBreak);
    public static AnalogInput shooterBeamBreak = new AnalogInput(portConstants.shooterBeamBreak);
    public BooleanSupplier intakeBeamBreakStatus = () -> (intakeBeamBreak.getAverageVoltage() > 0.5);
    public BooleanSupplier shooterBeamBreakStatus = () -> (shooterBeamBreak.getAverageVoltage() > 0.5);
}
