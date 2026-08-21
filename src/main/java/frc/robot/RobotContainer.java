// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIORealTalon;
import frc.robot.subsystems.Indexer.IndexerIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalon;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployIO;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployIOReal;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployIOSim;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOReal;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOSim;
import frc.robot.subsystems.StateManager.State;
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
import frc.robot.util.BLineAutoChooser;
import frc.robot.util.LoggedTunableNumber;
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

  // Robot demo bindings
  private final Trigger negativeManualHood = m_operatorController.leftBumper();
  private final Trigger positiveManualHood = m_operatorController.rightBumper();
  private final Trigger manualTurretPositive = m_operatorController.leftTrigger();
  private final Trigger manualTurretNegative = m_operatorController.rightTrigger();
  private final Trigger turretPos3 = m_driverController.povRight();

  // Official Bindings
  private final Trigger sotm = m_driverController.rightTrigger();
  private final Trigger intakeButton = m_driverController.leftTrigger();
  private final Trigger runIndexer = m_driverController.rightBumper();
  private final Trigger autoTrench = m_driverController.leftBumper();
  private final Trigger autoScore = m_driverController.x(); // No SOTM
  private final Trigger deployIntake = m_driverController.povDown();

  private final Trigger zeroDrive = m_driverController.y();
  private final Trigger reverseIntake = m_driverController.a();
  private final Trigger reverseIndexer = m_driverController.b();
  private final Trigger agitateIntake = m_driverController.povUp();

  // private final Trigger zeroButton = m_operatorController.a(); // Zero sequence
  private final Trigger toggleVision = m_operatorController.x();
  private final Trigger lockTurret = m_operatorController.b();
  // private final Trigger depoCorner = m_operatorController.povLeft();
  // private final Trigger feederCorner = m_operatorController.povRight();
  private final Trigger toggleAlliance = m_operatorController.y();
  private final Trigger highestSetpoint = m_operatorController.povUp();
  private final Trigger higherSetpoint = m_operatorController.povLeft();
  private final Trigger midSetpoint = m_operatorController.povRight();
  private final Trigger lowestSetpoint = m_operatorController.povDown();

  // Testing Bindings
  private final Trigger configure = m_driverController.povDown();

  private LoggedDashboardChooser<Command> autoChooser;
  private BLineAutoChooser bLineChooser;

  private LoggedTunableNumber driveMultiplier =
      new LoggedTunableNumber("Drive speed multiplier", 0.4);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Constants.FieldConstants.initConstants();
    switch (Constants.currentMode) {
      case REAL:
        intake = Intake.initialize(new IntakeIOTalon());
        deploy = IntakeDeploy.initialize(new IntakeDeployIOReal());
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
    // bLineChooser = drive.getAutoChooser(); // bline
    // bLineChooser.createAutos();

    // ok this is hellish
    // autoChooser = new LoggedDashboardChooser<>("Auto Chooser", bLineChooser.buildAutoChooser());

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
  // private void configureTestingButtons() {
  //     runIndexer.whileTrue(
  //         Commands.parallel(
  //             Commands.runOnce(() -> indexer.setRPSIndexer(40)),
  //             Commands.runOnce(() -> indexer.setVoltageTower(-12))));

  //     configure.onTrue(Commands.runOnce(() -> indexer.configure()));
  // }

  public void configureButtonBindings() {
    Trigger readyToShoot =
        new Trigger(
            () ->
                shooter.readyToShoot()
                    && shooterHood.readyToShoot()
                    && turret.readyToShoot()
                    && !OperationStates.behindTower);

    // Auto shoot
    readyToShoot.whileTrue(AutoCommands.shoot()).onFalse(AutoCommands.stopShoot());

    // Anti decapitation
    // Trigger inDecapitationZone = new Trigger(() -> OperationStates.inDecapitationZone);
    // inDecapitationZone.whileTrue(Commands.run(() -> shooterHood.setGoal(0), shooterHood));

    // Joystick drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY() * driveMultiplier.get(),
            () -> -m_driverController.getLeftX() * driveMultiplier.get(),
            () -> -m_driverController.getRightX() * driveMultiplier.get()));

    // Always aim at target
    // turret.setDefaultCommand(new AlwaysAim(drive, turret, stateManager));

    // SOTM
    sotm.whileTrue(
            Commands.parallel(
                AutoCommands.shootOnTheMove(),
                AutoCommands.intakeWithScaling(),
                AutoCommands.unjam(),
                DriveCommands.joystickDrive(
                    drive,
                    () ->
                        -m_driverController.getLeftY()
                            * (stateManager.getState() == State.SHOOTING ? 0.5 : 0.9)
                            * driveMultiplier.get(),
                    () ->
                        -m_driverController.getLeftX()
                            * (stateManager.getState() == State.SHOOTING ? 0.5 : 0.9)
                            * driveMultiplier.get(),
                    () ->
                        -m_driverController.getRightX()
                            * (stateManager.getState() == State.SHOOTING ? 0.4 : 0.7)
                            * driveMultiplier.get())))
        .onFalse(AutoCommands.afterShoot());

    // Auto trench
    // autoTrench.whileTrue(AutoCommands.underTrenchAssist());

    // Intake
    intakeButton.whileTrue(AutoCommands.intake());

    // m_operatorController
    //     .leftTrigger()
    //     .whileTrue(AutoCommands.intake())
    //     .onFalse(Commands.run(() -> intake.setGoalRPS(0), intake));

    // Run Indexer
    runIndexer.whileTrue(AutoCommands.shoot()).onFalse(AutoCommands.stopShoot());

    // Auto Score (no SOTM)
    autoScore
        .whileTrue(
            Commands.parallel(AutoCommands.autoScore(), Commands.run(() -> drive.stopWithX())))
        .onFalse(AutoCommands.afterShoot());

    // Toggle vision
    // toggleVision.whileTrue(Commands.runOnce(() -> vision.toggleShouldUpdate()));

    // Zero drive
    zeroDrive.onTrue(Commands.runOnce(() -> drive.zeroHeading()));

    // Toggle alliance
    toggleAlliance.onTrue(Commands.runOnce(() -> StateManager.toggleAlliance()));

    // Zero Sequence
    // zeroButton.whileTrue(AutoCommands.zeroSequence());

    // Lock turret
    lockTurret.onTrue(Commands.runOnce(() -> turret.lockTurret(), turret));

    // manual setpoints

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

    // outreach thingy
    negativeManualHood
        .whileTrue(Commands.run(() -> shooterHood.setVoltage(-1.5)))
        .onFalse(Commands.run(() -> shooterHood.setVoltage(0)));
    positiveManualHood
        .whileTrue(Commands.run(() -> shooterHood.setVoltage(1.5)))
        .onFalse(Commands.run(() -> shooterHood.setVoltage(0)));
    // turret setpoints
    manualTurretPositive
        .whileTrue(
            Commands.run(() -> turret.setVoltage(2 * m_operatorController.getLeftTriggerAxis())))
        .onFalse(Commands.run(() -> turret.setVoltage(0)));
    manualTurretNegative
        .whileTrue(
            Commands.run(() -> turret.setVoltage(-2 * m_operatorController.getRightTriggerAxis())))
        .onFalse(Commands.run(() -> turret.setVoltage(0)));

    highestSetpoint.onTrue(AutoCommands.setOutreachSetpoints(75));
    higherSetpoint.onTrue(AutoCommands.setOutreachSetpoints(40));
    midSetpoint.onTrue(AutoCommands.setOutreachSetpoints(30));
    lowestSetpoint.onTrue(AutoCommands.setOutreachSetpoints(13));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.x`
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();

    // return Commands.sequence(
    //     drive.pathBuilder.build(new Path("Rembrandts P1")),
    //     drive.pathBuilder.build(new Path("Rembrandts P2")));
  }

  public Command stopMechanisms() {
    return Commands.parallel(
        Commands.runOnce(() -> shooterHood.setGoal(0)),
        Commands.runOnce(() -> shooter.setVelocityRPS(0)),
        AutoCommands.stopShoot());
  }

  public void startTimer() {
    stateManager.startMatchTimer();
  }

  public void onEnable() {
    turret.setBrake();
  }

  public void onDisable() {
    turret.setCoast();
  }
}
