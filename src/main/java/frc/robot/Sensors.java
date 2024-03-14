package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.portConstants;
import frc.robot.subsystems.shooterSubystem;
public class Sensors {
    public static AnalogInput shooterBeamBreak = new AnalogInput(portConstants.shooterBeamBreak);
    public BooleanSupplier shooterBeamBreakStatus = () -> (shooterBeamBreak.getAverageVoltage() == 0 );
}

