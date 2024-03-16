package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.portConstants;
import frc.robot.subsystems.shooterSubystem;
public class Sensors {
    public static AnalogInput shooterBeamBreak = new AnalogInput(portConstants.shooterBeamBreak);
    public static BooleanSupplier shooterBeamBreakStatus = () -> (shooterBeamBreak.getValue() < 10 );
    public BooleanSupplier shooterBeamBreakStatusINV = () -> (shooterBeamBreak.getValue() > 10 );

    public static boolean getBeamBreakStatus(){
        if(shooterBeamBreak.getValue() < 10){
            return true;
        } else {
            return false;
        }
    }
}

