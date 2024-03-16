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
    private final DriveSubsystem drivebase = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private intakeSubsystem m_IntakeSubsystem = new intakeSubsystem();
    private indexerSubystem m_IndexerSubystem = new indexerSubystem();
    private shooterSubystem m_ShooterSubystem = new shooterSubystem();
    private Auto m_Auto = new Auto();
    private Sensors m_Sensors = new Sensors();
    private climberSubsystem m_ClimberSubsystem = new climberSubsystem();
    XboxController driverXbox = new XboxController(OperatorConstants.kDriverControllerPort);
    public static XboxController operatorXbox = new XboxController(OperatorConstants.kOperatorControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        // Configure the trigger bindings

        //Autonmous drive forward

        // AUTO COMMANDS

          Command intake =   new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::StartIntake),  
                   new InstantCommand(m_IndexerSubystem::startIndexer),
                   new WaitCommand(.5),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatus).withTimeout(5),
                   new InstantCommand(m_IntakeSubsystem::stopIntake),
                   new InstantCommand(m_IndexerSubystem::reverseIndexer),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatusINV),
                   new InstantCommand(m_IndexerSubystem::stopIndexer));
                   
        Command shoot = new SequentialCommandGroup(new InstantCommand(m_IndexerSubystem::startIndexer),
                new WaitCommand(2),
                new InstantCommand(m_ShooterSubystem::stopShooter));

        Command spinUpShooter = new InstantCommand(m_ShooterSubystem::StartShooter);

        //Start PATH PLANNER
        drivebase.setupPathPlanner();
                NamedCommands.registerCommand("spinUpShooter", spinUpShooter);
        NamedCommands.registerCommand("shoot", shoot);
        NamedCommands.registerCommand("intake", intake);
        SmartDashboard.putString("Intake", "OFF");
        SmartDashboard.putString("Shooter", "OFF");
                SmartDashboard.putString("Indexer", "OFF");

      

        configureBindings();

        
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(),0.05 ),
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),0.05),
                () -> driverXbox.getRightX(),
                () -> -driverXbox.getRightY());
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05),
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), 0.05),
                () -> driverXbox.getRawAxis(4));

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> driverXbox.getLeftY(),
                () -> driverXbox.getLeftX(),
                () -> driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    }
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
        // Main intake command
        NamedCommands.registerCommand(null, null);
        new JoystickButton(operatorXbox, XboxController.Button.kLeftBumper.value).onTrue(
                new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::StartIntake),  
                   new InstantCommand(m_IndexerSubystem::startIndexer),
                   new WaitCommand(.5),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatus).withTimeout(5),
                   new InstantCommand(m_IntakeSubsystem::stopIntake),
                   new InstantCommand(m_IndexerSubystem::reverseIndexer),
                   new WaitUntilCommand(m_Sensors.shooterBeamBreakStatusINV),
                   new InstantCommand(m_IndexerSubystem::stopIndexer)
   
           ));
        // Start Shooter Command
        new JoystickButton(operatorXbox, XboxController.Button.kA.value).onTrue(
                new InstantCommand(m_ShooterSubystem::StartShooter));
        // Stop Shooter Manual Command
        new JoystickButton(operatorXbox, XboxController.Button.kB.value).onTrue(
                new InstantCommand(m_ShooterSubystem::stopShooter));
        // Shoot the Ball
        new JoystickButton(operatorXbox, XboxController.Button.kRightBumper.value).onTrue(
                new SequentialCommandGroup(new InstantCommand(m_IndexerSubystem::startIndexer),
                        new WaitCommand(2),
                        new InstantCommand(m_ShooterSubystem::stopShooter),
                        new InstantCommand(m_IndexerSubystem::stopIndexer))

        );
        // Reverse Intake
        new JoystickButton(operatorXbox, XboxController.Button.kY.value).onTrue(
                new SequentialCommandGroup(new InstantCommand(m_IntakeSubsystem::reverseIntake),
                new InstantCommand(m_IndexerSubystem::reverseIndexer)));
        // Manual Commands
        Trigger dpadUpTrigger = new Trigger(() -> operatorXbox.getPOV() == 0);
        dpadUpTrigger.onTrue(new InstantCommand(m_IntakeSubsystem::manualIntake));
        Trigger dpadRighTrigger = new Trigger(() -> operatorXbox.getPOV() == 90);
        dpadRighTrigger.onTrue(new InstantCommand(m_IndexerSubystem::manualIndexer));
        Trigger dpadDownTrigger = new Trigger(() -> operatorXbox.getPOV() == 180);
        dpadDownTrigger.onTrue(new InstantCommand(m_IntakeSubsystem::stopIntake));
        Trigger dpadLefTrigger = new Trigger(() -> operatorXbox.getPOV() == 270);
        dpadLefTrigger.onTrue(new InstantCommand(m_IndexerSubystem::stopIndexer));
    }
    public void dataout () {    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
}
