// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIORealTalon;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployIO;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOReal;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOSim;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.StateManager.StateManager.OperationStates;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkFlex;
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
  private final Shooter shooter;
  private final Indexer indexer;
  private final Turret turret;
  private final Drive drive;
  private final Vision vision;
  private final ShooterHood shooterHood;
  private final StateManager stateManager;
  private final Intake intake;
  private final IntakeDeploy deploy;

  // Controllers
  private final CommandXboxController m_driverController = Constants.OIConstants.driverController;
  private final CommandXboxController m_operatorController =
      Constants.OIConstants.operatorController;

  // Official Bindings
  private final Trigger sotm = m_driverController.rightTrigger();
  private final Trigger autoTrench = m_driverController.leftBumper();
  private final Trigger shoot = m_driverController.rightBumper();
  private final Trigger intakeButton = m_driverController.leftTrigger();
  private final Trigger autoScore = m_driverController.a(); // No SOTM
  private final Trigger toggleVision = m_driverController.x();
  private final Trigger zeroDrive = m_driverController.y();

  private final Trigger zeroButton = m_operatorController.a(); // Zero sequence
  private final Trigger lockTurret = m_operatorController.b();
  private final Trigger depoCorner = m_operatorController.povLeft();
  private final Trigger feederCorner = m_operatorController.povRight();
  private final Trigger agitateIntake = m_operatorController.povUp();
  private final Trigger deployIntake = m_operatorController.povDown();
  private final Trigger reverseIntake = m_operatorController.leftBumper();
  private final Trigger reverseIndexer = m_operatorController.rightBumper();
  // Operator Triggers already bound to manual intake deploy

  // Testing Bindings
  private final Trigger configure = m_driverController.povDown();

  private LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // there's probably a better way to do this
    Constants.FieldConstants.initConstants();
    switch (Constants.currentMode) {
      case REAL:
        intake = Intake.initialize(new IntakeIOReal());
        deploy = IntakeDeploy.initialize(new IntakeDeployIOSim());
        shooter = Shooter.initialize(new ShooterIOReal());
        indexer = Indexer.initialize(new IndexerIORealTalon());
        turret = Turret.initialize(new TurretIOReal());
        shooterHood = ShooterHood.initialize(new ShooterHoodIOReal());
        drive =
            Drive.initialize(
                new GyroIOPigeon2(),
                new ModuleIOSparkFlex(0),
                new ModuleIOSparkFlex(1),
                new ModuleIOSparkFlex(2),
                new ModuleIOSparkFlex(3));
        // drive =
        //     Drive.initialize(
        //         new GyroIO() {},
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim(),
        //         new ModuleIOSim());
        vision = Vision.initialize(new VisionIOReal(0), new VisionIOReal(1), new VisionIOReal(2));
        break;

      case SIM:
        intake = Intake.initialize(new IntakeIOSim());
        deploy = IntakeDeploy.initialize(new IntakeDeployIOSim());
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
        intake = Intake.initialize(new IntakeIO() {});
        deploy = IntakeDeploy.initialize(new IntakeDeployIO() {});
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

    stateManager = StateManager.initalize();

    // Register named commands
    NamedCommandManager.registerNamedCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // Configure the trigger bindings
    configureButtonBindings();
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
  private void configureTestingButtons() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    intakeButton.whileTrue(AutoCommands.intake());

    autoScore.whileTrue(AutoCommands.autoScore());

    // sotm.whileTrue(
    //     AutoCommands.movingAlign()
    //         .alongWith(
    //             DriveCommands.joystickDrive(
    //                 drive,
    //                 () -> -m_driverController.getLeftY() / 2.0,
    //                 () -> -m_driverController.getLeftX() / 2.0,
    //                 () -> -m_driverController.getRightX() / 2.0)));

    // autoScore.whileTrue(AutoCommands.autoScore());

    // shoot.whileTrue(AutoCommands.shoot()).onFalse(AutoCommands.stopShoot());

    autoTrench.whileTrue(AutoCommands.underTrenchAssist());

    configure.onTrue(Commands.sequence(Commands.runOnce(() -> turret.configure())));

    zeroButton.onTrue(AutoCommands.zeroSequence());

    agitateIntake.whileTrue(AutoCommands.agitateIntake());

    // retractIntake.whileTrue(AutoCommands.retractIntake());

    deployIntake.whileTrue(AutoCommands.deployIntake());

    m_operatorController
        .povUp()
        .whileTrue(Commands.run(() -> drive.sysIdDynamic(Direction.kForward)));
    m_operatorController
        .povDown()
        .whileTrue(Commands.run(() -> drive.sysIdDynamic(Direction.kReverse)));
    m_operatorController
        .povLeft()
        .whileTrue(Commands.run(() -> drive.sysIdQuasistatic(Direction.kForward)));
    m_operatorController
        .povRight()
        .whileTrue(Commands.run(() -> drive.sysIdQuasistatic(Direction.kReverse)));

    // retractIntake.whileTrue(
    //     Commands.startEnd(
    //         () -> deploy.setDeployVoltage(3), () -> deploy.setDeployVoltage(0), deploy));

    // shooterButton.whileTrue(AutoCommands.setShooterRPS(50));

    // Dynamic shooting
    // m_operatorController.leftBumper().whileTrue(AutoCommands.setShooterVelocityDynamic());

    // Turret Bindings
    // turretOne.whileTrue(
    //     Commands.startEnd(() -> turret.setVoltage(4), () -> turret.setVoltage(0), turret));
    // turretTwo.whileTrue(
    //     Commands.startEnd(() -> turret.setVoltage(-4), () -> turret.setVoltage(0), turret));
    // zeroTurretTrigger.onTrue(Commands.runOnce(() -> turret.zero()));

    // shooterHoodUp
    //     .whileTrue(Commands.run(() -> shooterHood.setGoal(1), shooterHood));
    // shooterHoodDown
    //     .whileTrue(Commands.run(() -> shooterHood.setGoal(0.25), shooterHood));
    // Dynamic shooter hood
    // m_driverController.leftTrigger().whileTrue(AutoCommands.setShooterHoodDynamic());

    // m_operatorController
    //     .a()
    //     .whileTrue(
    //         AutoCommands.turretAimAtTarget()
    //             .alongWith(
    //                 Commands.run(() -> shooterHood.setGoal(ShooterHoodIOReal.hoodAngle.get())))
    //             .alongWith(
    //                 Commands.startEnd(
    //                     () -> shooter.setVelocityRPS(ShooterIOReal.shooterRPS.get()),
    //                     () -> shooter.setVelocityRPS(0))));
  }

  public void configureButtonBindings() {
    Trigger readyToShoot =
        new Trigger(
            () -> shooter.readyToShoot() && shooterHood.readyToShoot() && turret.readyToShoot());

    Trigger inDecapitationZone = new Trigger(() -> OperationStates.inDecapitationZone);

    // Auto shoot
    readyToShoot.whileTrue(AutoCommands.shoot()).onFalse(AutoCommands.stopShoot());

    // Anti decapitation
    inDecapitationZone.whileTrue(Commands.run(() -> shooterHood.setGoal(0)));

    // Joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // SOTM
    sotm.whileTrue(
            Commands.parallel(
                AutoCommands.shootOnTheMove(),
                AutoCommands.intake(),
                DriveCommands.joystickDrive(
                    drive,
                    () -> -m_driverController.getLeftY() / 2.0,
                    () -> -m_driverController.getLeftX() / 2.0,
                    () -> -m_driverController.getRightX() / 3.0)))
        .onFalse(Commands.runOnce(() -> shooter.resetShooter()));

    // Auto trench
    autoTrench.whileTrue(AutoCommands.underTrenchAssist());

    // Intake
    intakeButton.whileTrue(AutoCommands.intake());

    // Run Indexer
    shoot.whileTrue(AutoCommands.shoot()).onFalse(AutoCommands.stopShoot());

    // Auto Score (no SOTM)
    autoScore.whileTrue(
        Commands.parallel(AutoCommands.autoScore(), Commands.runOnce(() -> drive.stopWithX())));

    // Toggle vision
    toggleVision.whileTrue(Commands.runOnce(() -> vision.toggleShouldUpdate()));

    // Zero drive
    zeroDrive.onTrue(Commands.runOnce(() -> drive.zeroHeading()));

    // Zero Sequence
    zeroButton.whileTrue(AutoCommands.zeroSequence());

    // Lock turret
    lockTurret.onTrue(Commands.runOnce(() -> turret.lockTurret(), turret));

    // Corner setpoints
    depoCorner.whileTrue(AutoCommands.depoCornerAlign());
    feederCorner.whileTrue(AutoCommands.feederCornerAlign());

    // Deployable intake
    deployIntake.whileTrue(AutoCommands.deployIntake());
    agitateIntake.whileTrue(AutoCommands.agitateIntake());

    // Reverse intake
    reverseIntake.whileTrue(AutoCommands.reverseIntake());

    // Reverse indexer
    reverseIndexer.whileTrue(AutoCommands.reverseIndexer()).onFalse(AutoCommands.stopShoot());

    // add climber here
    // m_operatorController.povUp().whileTrue(Comamnds.startEnd(() -> ))
    // m_operatorController.povDown()

    // add manual setpoints here
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
