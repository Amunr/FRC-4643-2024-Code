package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.portConstants;
import frc.robot.subsystems.shooterSubystem;
public class Sensors {
    public static AnalogInput shooterBeamBreak = new AnalogInput(portConstants.shooterBeamBreak);
    public static AnalogInput intakeBeamBreak = new AnalogInput(portConstants.intakeBeamBreak);
    public BooleanSupplier shooterBeamBreakStatus = () -> (shooterBeamBreak.getValue() < 10 );
    public BooleanSupplier shooterBeamBreakStatusINV = () -> (shooterBeamBreak.getValue() > 10 );
    public BooleanSupplier intakeBeamBreakStatus = () -> (intakeBeamBreak.getValue() < 10);
    public BooleanSupplier intakeBeamBreakStatusINV = () -> (intakeBeamBreak.getValue() > 10 );
    public boolean gameObject = true; 
    public static boolean getBeamBreakStatus(){
        if(shooterBeamBreak.getValue() < 10){
            return true;
        } else {
            return false;
        }

    }
    public void yesGameobject(){

        gameObject = true;
    }
    public void noGameObject(){
        gameObject = false;
    }
}
