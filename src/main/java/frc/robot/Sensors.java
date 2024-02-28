package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.portConstants;
public class Sensors {
    public static AnalogInput intakeBeamBreak = new AnalogInput(portConstants.intakeBeamBreak);
    public static AnalogInput shooterBeamBreak = new AnalogInput(portConstants.shooterBeamBreak);

}
