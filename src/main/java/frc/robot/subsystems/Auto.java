package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Auto extends SubsystemBase{
    public static Command getAutonomousCommand(){
        //PathPlannerPath pathLeft = PathPlannerPath.fromPathFile("crossTheLineLeft");
       // return AutoBuilder.followPath(pathLeft);
        return new PathPlannerAuto("crossTheLineLeft");
        
    }
}   