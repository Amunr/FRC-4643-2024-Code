// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Auto;
// import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.climberSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubystem;
import frc.robot.subsystems.indexerSubystem;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Time;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Sensors;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer { 
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem drivebase = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "Swerve"));
    private intakeSubsystem m_IntakeSubsystem = new intakeSubsystem();
    private indexerSubystem m_IndexerSubystem = new indexerSubystem();
    private shooterSubystem m_ShooterSubystem = new shooterSubystem();
  //  private Auto m_Auto = new Auto();
    private Sensors m_Sensors = new Sensors();
    private climberSubsystem m_ClimberSubsystem = new climberSubsystem();
    XboxController driverXbox = new XboxController(OperatorConstants.kDriverControllerPort);
    public static XboxController operatorXbox = new XboxController(OperatorConstants.kOperatorControllerPort);
    public static XboxController maintenanceController  = new XboxController(OperatorConstants.kMaintenaceControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        // Configure the trigger bindings

        //Autonmous drive forward


        // AUTO COMMANDS
        //   Command intake =   new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::StartIntake),  
        //            new InstantCommand(m_IndexerSubystem::startIndexer),
        //            new WaitCommand(.5),
        //            new WaitUntilCommand(m_Sensors.shooterBeamBreakStatus).withTimeout(5),
        //            new InstantCommand(m_IntakeSubsystem::stopIntake),
        //            new InstantCommand(m_IndexerSubystem::reverseIndexer),
        //            new WaitUntilCommand(m_Sensors.shooterBeamBreakStatusINV),
        //            new InstantCommand(m_IndexerSubystem::stopIndexer));

        // Command shoot = new SequentialCommandGroup(new InstantCommand(m_IndexerSubystem::startIndexer),
        //         new WaitCommand(2),
        //         new InstantCommand(m_ShooterSubystem::stopShooter));

        // Command spinUpShooter = new InstantCommand(m_ShooterSubystem::StartShooter);

        //Start PATH PLANNER
        drivebase.setupPathPlanner();
        //         NamedCommands.registerCommand("spinUpShooter", spinUpShooter);
        // NamedCommands.registerCommand("shoot", shoot);
        // NamedCommands.registerCommand("intake", intake);
        // SmartDashboard.putString("Intake", "OFF");
        // SmartDashboard.putString("Shooter", "OFF");
        //         SmartDashboard.putString("Indexer", "OFF");


        //Drive Controls
        configureBindings();


        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY()/ 0,0.05),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX() ,0.05),
                () -> (driverXbox.getRightX()/2),
                () -> (-driverXbox.getRightY()/2));
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), 0.05),
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), 0.05),
                () -> driverXbox.getRawAxis(4));
        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () ->  driverXbox.getLeftY(),
                () ->  driverXbox.getLeftX(),
                () -> driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
        
    }
   final Command driveLowAuto = drivebase.autoDrive(0.1, 0.0, 0.0, 0.0);
     final Command driveLowAng = drivebase.autoDriveAng(0.1, 0.0, 0.0);
    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
                //Release Game object / Reverse intake
                new Trigger(() -> operatorXbox.getLeftTriggerAxis() > 0.3).whileTrue(new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::reverseIntake),
                new InstantCommand(m_IndexerSubystem::reverseIndexer),
                new InstantCommand(m_Sensors::noGameObject))).onFalse(new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::stopIntake),
           new InstantCommand(m_IndexerSubystem::stopIndexer))); ;
                // Prepare to fire
                new Trigger(() -> operatorXbox.getRightTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_ShooterSubystem::StartShooter)).onFalse(new InstantCommand(m_ShooterSubystem::stopShooter)); 
        // Intake
                new JoystickButton(operatorXbox, XboxController.Button.kLeftBumper.value).whileTrue(
                new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::StartIntake),  
                   new InstantCommand(m_IndexerSubystem::startIndexer),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatus),
                   new InstantCommand(m_IntakeSubsystem::stopIntake),
                   new InstantCommand(m_IndexerSubystem::reverseIndexer),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatusINV),
                   new InstantCommand(m_IndexerSubystem::stopIndexer),
                   new InstantCommand(m_Sensors::yesGameobject)

           )).onFalse(new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::stopIntake),
           new InstantCommand(m_IndexerSubystem::stopIndexer))); 
               // Shoot the Ball
        new JoystickButton(operatorXbox, XboxController.Button.kRightBumper.value).whileTrue(
                new SequentialCommandGroup(new InstantCommand(m_IndexerSubystem::startIndexer),
                new InstantCommand(m_Sensors::noGameObject))).onFalse(
                new SequentialCommandGroup(new InstantCommand(m_ShooterSubystem::stopShooter),
                        new InstantCommand(m_IndexerSubystem::stopIndexer)));
        // Start Shooter Command



                //MANUAL

        // Stop Shooter Manual Command
        // new JoystickButton(operatorXbox, XboxController.Button.kB.value).onTrue(
        //         new InstantCommand(m_ShooterSubystem::stopShooter));
        //Start INDEXER FAST COMMAND
        // new JoystickButton(operatorXbox, XboxController.Button.kX.value).onTrue(new InstantCommand(m_IndexerSubystem::startIndexer));


        // NEW manual
          new Trigger(() -> operatorXbox.getPOV() == 0).whileTrue(new InstantCommand(m_IntakeSubsystem::manualIntake)).onFalse(new InstantCommand(m_IntakeSubsystem::stopIntake));
           new Trigger(() -> operatorXbox.getPOV() == 90).whileTrue(new InstantCommand(m_IndexerSubystem::manualIndexer)).onFalse(new InstantCommand(m_IntakeSubsystem::stopIntake));
        new Trigger(() -> operatorXbox.getPOV() == 180).whileTrue(new InstantCommand(m_IntakeSubsystem::reverseIntake)).onFalse(new InstantCommand(m_IntakeSubsystem::stopIntake));
         new Trigger(() -> operatorXbox.getPOV() == 270).whileTrue(new InstantCommand(m_IndexerSubystem::reverseIndexer)).onFalse(new InstantCommand(m_IntakeSubsystem::stopIntake));
        //STOP ALL
                new JoystickButton(operatorXbox, XboxController.Button.kB.value).onTrue(new SequentialCommandGroup(
                new InstantCommand(m_ShooterSubystem::stopShooter), 
                new InstantCommand(m_IntakeSubsystem::stopIntake),
                new InstantCommand(m_IndexerSubystem::stopIndexer)
                ));
        }

        Command shoot = new SequentialCommandGroup(new InstantCommand(m_ShooterSubystem::StartShooter),
                        new WaitCommand(2), new InstantCommand(m_IndexerSubystem::startIndexer), new WaitCommand(3),
                        new InstantCommand(m_ShooterSubystem::stopShooter),
                        new InstantCommand(m_IndexerSubystem::stopIndexer), driveLowAng, driveLowAuto);

          public Command getAutonomousCommand() {
           return driveLowAuto;
           }
           public  Command getAutonomousCommandB(){
                return driveLowAng;
           }
           public Command getAutonomousCommandC(){
                return shoot;
           }

    public void dataout () { 

       }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
}