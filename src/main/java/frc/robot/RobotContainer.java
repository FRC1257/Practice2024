// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.GroundIntake.GroundIntake;
import frc.robot.subsystems.GroundIntake.GroundIntakeIO;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSim;
import frc.robot.subsystems.GroundIntake.GroundIntakeIOSparkMax;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmIO;
import frc.robot.subsystems.pivotArm.PivotArmIOSim;
import frc.robot.subsystems.pivotArm.PivotArmIOSparkMax;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.CommandSnailController;
import static frc.robot.util.drive.DriveControls.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final PivotArm pivotArm;
  private final GroundIntake groundIntake;
  private final Indexer indexer;

  private Mechanism2d mech = new Mechanism2d(3, 3);

  // Controllers
  private final CommandSnailController driver = new CommandSnailController(0);
  private final CommandSnailController operator = new CommandSnailController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean isBlue = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(
          new GyroIOReal(),
            new ModuleIOSparkMax(0), // Front Left
            new ModuleIOSparkMax(1), // Front Right
            new ModuleIOSparkMax(2), // Back left
            new ModuleIOSparkMax(3), // Back right
            new VisionIOPhoton()
        );
        shooter = new Shooter(
          new ShooterIOSparkMax()
        );
        pivotArm = new PivotArm(
          new PivotArmIOSparkMax()
        );
        groundIntake = new GroundIntake(
          new GroundIntakeIOSparkMax()
        );
        indexer = new Indexer(
          new IndexerIOSparkMax()
        );
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim());
        shooter = new Shooter(
          new ShooterIOSim()
        );
        pivotArm = new PivotArm(
          new PivotArmIOSim()
        );
        groundIntake = new GroundIntake(
          new GroundIntakeIOSim()
        );
        indexer = new Indexer(
          new IndexerIOSim()
        );
        break;
      case TEST:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOSim()
        );
        shooter = new Shooter(
          new ShooterIOSim()
        );
        pivotArm = new PivotArm(
          new PivotArmIOSim()
        );
        groundIntake = new GroundIntake(
          new GroundIntakeIOSim()
        );
        indexer = new Indexer(
          new IndexerIOSim()
        );
        break;

      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new VisionIO() {
            });
        shooter = new Shooter(
          new ShooterIO() {}
        );
        pivotArm = new PivotArm(
          new PivotArmIO() {}
        );
        groundIntake = new GroundIntake(
          new GroundIntakeIO() {}
        );
        indexer = new Indexer(
          new IndexerIO() {}
        );
        break;
    }

    // Set up robot state manager
    MechanismRoot2d root = mech.getRoot("pivot", 1, 0.5);
    
    pivotArm.setMechanism(root.append(pivotArm.getMechanism()));

    // add subsystem mechanisms
    SmartDashboard.putData("Arm Mechanism", mech);

    isBlue = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue);

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    drive.setDefaultCommand( // change state here
        DriveCommands.joystickDrive(
            drive,
            DRIVE_FORWARD,
            DRIVE_STRAFE,
            DRIVE_ROTATE));

        DRIVE_SLOW.onTrue(new InstantCommand(DriveCommands::toggleSlowMode));

        shooter.setDefaultCommand(shooter.runVoltage(0)); //redefine later

        pivotArm.setDefaultCommand(pivotArm.manualCommand(0)); //redefine later

        groundIntake.setDefaultCommand(groundIntake.runSpeedCommand(0));

        indexer.setDefaultCommand(indexer.runSpeedCommand(0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
