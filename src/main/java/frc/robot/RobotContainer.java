// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOReal;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOReal;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOSim;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIORedux;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.NamedCommandManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Shooter shooter;
  private Indexer indexer;
  private Turret turret;
  private Drive drive;
  private final Vision vision;
  private ShooterHood shooterHood;

  // Controllers
  private final CommandXboxController m_driverController = Constants.OIConstants.driverController;
  private final CommandXboxController m_operatorController =
      Constants.OIConstants.operatorController;

  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // there's probably a better way to do this
    Constants.FieldConstants.initConstants();
    switch (Constants.currentMode) {
      case REAL:
        shooter = Shooter.initialize(new ShooterIOReal());
        indexer = Indexer.initialize(new IndexerIOReal());
        turret = Turret.initialize(new TurretIOReal());
        shooterHood = ShooterHood.initialize(new ShooterHoodIOReal());
        drive =
            Drive.initialize(
                new GyroIORedux(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision = Vision.initialize(new VisionIOReal(0));
        break;

      case SIM:
        shooter = Shooter.initialize(new ShooterIOSim());
        indexer = Indexer.initialize(new IndexerIOSim());
        turret = Turret.initialize(new TurretIOSim());
        shooterHood = ShooterHood.initialize(new ShooterHoodIOSim());
        drive =
            Drive.initialize(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = Vision.initialize(new VisionIOSim());
        break;

      default:
        shooter = Shooter.initialize(new ShooterIO() {});
        indexer = Indexer.initialize(new IndexerIO() {});
        turret = Turret.initialize(new TurretIO() {});
        shooterHood = ShooterHood.initialize(new ShooterHoodIO() {});
        drive =
            Drive.initialize(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision = Vision.initialize(new VisionIO[] {});
        break;
    }

    // Register named commands
    NamedCommandManager.registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // Configure the trigger bindings
    configureBindings();
  }

  public boolean readyToShoot() {
    return shooter.readyToShoot() && shooterHood.readyToShoot() && turret.readyToShoot();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // For sim
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Shooter Bindings
    m_operatorController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setVelocityRPS(ShooterIOReal.RPM.get()),
                () -> shooter.setVelocityRPS(0)));

    m_operatorController.leftBumper().whileTrue(AutoCommands.setShooterVelocityDynamic());

    // m_operatorController
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.startEnd(() -> shooter.setVoltage(6), () -> shooter.setVoltage(0),
    // shooter));

    // Turret Bindings
    m_operatorController.povLeft().onTrue(Commands.runOnce(() -> turret.setGoal(0.5)));
    m_operatorController.povRight().onTrue(Commands.runOnce(() -> turret.setGoal(0.1)));
    m_operatorController.povDown().whileTrue(new AimAtHub(drive, turret));
    m_driverController.x().onTrue(Commands.runOnce(() -> turret.zero()));

    // Indexer Bindings
    m_operatorController
        .povUp()
        .whileTrue(
            Commands.startEnd(
                () -> indexer.setVoltageTower(6), () -> indexer.setVoltageTower(0), indexer));

    // Needs to be tested
    m_operatorController.a().whileTrue(AutoCommands.zeroSequence());

    // Shooter Hood Bindings. Need to be tested.
    m_driverController
        .rightBumper()
        .whileTrue(Commands.run(() -> shooterHood.setGoal(1), shooterHood));
    m_driverController
        .leftBumper()
        .whileTrue(Commands.run(() -> shooterHood.setGoal(0.25), shooterHood));

    m_driverController.leftTrigger().whileTrue(AutoCommands.setShooterHoodDynamic());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.x`
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }
}
