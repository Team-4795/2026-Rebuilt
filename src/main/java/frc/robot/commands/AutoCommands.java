package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeDeploy.IntakeDeployConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.StateManager.State;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Indexer indexer = Indexer.getInstance();
  private static Turret turret = Turret.getInstance();
  private static ShooterHood hood = ShooterHood.getInstance();
  private static Intake intake = Intake.getInstance();
  private static IntakeDeploy deploy = IntakeDeploy.getInstance();
  private static StateManager manager = StateManager.getInstance();

  private AutoCommands() {}

  public static Command retractIntake() {
    return Commands.runOnce(() -> deploy.setGoal(IntakeDeployConstants.stowPosition - 0.05));
  }

  public static Command deployIntake() {
    return Commands.runOnce(() -> deploy.setGoal(IntakeDeployConstants.intakePosition));
  }

  public static Command liftIntake() {
    return Commands.runOnce(() -> deploy.setGoal(0.04));
  }

  public static Command agitateIntake() {
    return Commands.parallel(
        Commands.repeatingSequence(
            Commands.runOnce(() -> deploy.setGoal(0.04)),
            Commands.waitSeconds(0.5),
            deployIntake(),
            Commands.waitSeconds(0.5)));
  }

  public static Command agitateIntakeAuto() {
    return Commands.repeatingSequence(
        Commands.runOnce(() -> deploy.setGoal(0.04)),
        Commands.waitSeconds(1),
        deployIntake(),
        Commands.waitSeconds(1));
  }

  // public static Command autoAgitate() {
  //   return Commands.sequence(Commands.waitSeconds(3), agitateIntake())
  //       .onlyWhile(() -> !OperationStates.isIntaking)
  //       .repeatedly();
  // }

  public static Command intake() {
    return Commands.run(() -> intake.setIntakeVoltage(12), intake);
  }

  public static Command reverseIntake() {
    return Commands.run(() -> intake.setIntakeVoltage(-12), intake);
  }

  public static Command shootOnTheMove() {
    return new ShootOnTheMove(drive, turret, shooter, hood, manager);
  }

  public static Command newSOTM() {
    return new NewSOTM(drive, turret, shooter, hood, manager);
  }

  public static Command autonomousSOTM() {
    return new AutonomousSOTM(drive, turret, shooter, hood, manager);
  }

  public static Command autoScore() {
    return new AimAtTarget(drive, turret, shooter, hood, manager);
  }

  // public static Command interpolationMapTesting() {
  //   return Commands.parallel(turretAimAtTarget(), setShooterRPS(ShooterIOReal.shooterRPS.get()));
  // }

  public static Command shoot() {
    return Commands.either(
        Commands.parallel(
            Commands.runOnce(() -> indexer.setVoltageIndexer(-11)),
            Commands.runOnce(() -> indexer.setVoltageTower(-12))),
        Commands.parallel(
            Commands.runOnce(() -> indexer.setVoltageIndexer(-8)),
            Commands.runOnce(() -> indexer.setVoltageTower(-11))),
        () -> manager.getState() == State.SHOOTING);
  }

  public static Command reverseIndexer() {
    return Commands.parallel(
        Commands.runOnce(() -> indexer.setVoltageIndexer(11)),
        Commands.runOnce(() -> indexer.setVoltageTower(9)));
  }

  public static Command stopShoot() {
    return Commands.parallel(
        Commands.runOnce(() -> indexer.setVoltageIndexer(0)),
        Commands.runOnce(() -> indexer.setVoltageTower(0)));
  }

  public static Command zeroSequence() { // if it doesn't work check the motor limits
    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> turret.setVoltage(-2), turret),
            Commands.waitSeconds(1.5), // change
            Commands.runOnce(() -> turret.zero(), turret),
            Commands.runOnce(() -> turret.setVoltage(0), turret)),
        Commands.sequence(
            Commands.runOnce(() -> hood.setVoltage(3), hood),
            Commands.waitSeconds(1.5), // change
            Commands.runOnce(() -> hood.zero(), hood),
            Commands.runOnce(() -> hood.setVoltage(0), hood)),
        Commands.sequence(
            Commands.runOnce(() -> deploy.setDeployVoltage(3), deploy),
            Commands.waitSeconds(1.5),
            Commands.runOnce(() -> deploy.setDeployVoltage(0), deploy),
            Commands.runOnce(() -> deploy.zero(), deploy)));
  }

  public static Command underTrenchAssist() {
    return new UnderTrench(drive);
  }

  public static Command setShooterRPS(double RPS) {
    return Commands.startEnd(
        () -> shooter.setVelocityRPS(RPS), () -> shooter.setVelocityRPS(0), shooter);
  }

  public static Command setHoodAngle(double angle) {
    return Commands.runOnce(() -> hood.setGoal(angle), hood);
  }

  // Dynamic Interpolating Tree Commands
  public static Command setShooterVelocityDynamic() {
    return Commands.either(
        Commands.startEnd(
            () ->
                shooter.setVelocityRPS(
                    ShooterConstants.shooterVelocityHubMap.get(
                        Drive.getInstance().getTurretDistanceToTarget())),
            () -> shooter.setVelocityRPS(0)),
        Commands.startEnd(
            () ->
                shooter.setVelocityRPS(
                    ShooterConstants.shooterVelocityShuttlingMap.get(
                        Drive.getInstance().getTurretDistanceToTarget())),
            () -> shooter.setVelocityRPS(0)),
        () -> manager.getState().equals(State.SHOOTING));
  }

  public static Command setShooterHoodDynamic() {
    return Commands.either(
        Commands.run(
            () ->
                hood.setGoal(
                    ShooterHoodConstants.shooterHoodHubMap.get(
                        Drive.getInstance().getTurretDistanceToTarget()))),
        Commands.run(
            () ->
                hood.setGoal(
                    ShooterHoodConstants.shooterHoodShuttlingMap.get(
                        Drive.getInstance().getTurretDistanceToTarget()))),
        () -> manager.getState().equals(State.SHOOTING));
  }

  // Keep shooter and hood aimed for a period after to not miss last shots
  public static Command afterShoot() {
    return Commands.parallel(
        stopShoot(),
        Commands.runOnce(() -> shooter.setVelocityRPS(0)),
        Commands.runOnce(() -> hood.setGoal(0)),
        Commands.runOnce(() -> shooter.resetShooter()));
  }

  public static Command unjam() {
    return Commands.repeatingSequence(
            Commands.waitSeconds(3.5),
            Commands.runOnce(() -> indexer.setVoltageIndexer(0), indexer),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> indexer.setVoltageIndexer(12), indexer),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> indexer.setVoltageIndexer(0), indexer),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> indexer.setVoltageIndexer(-12), indexer))
        .onlyWhile(() -> !indexer.didCurrentSpike());
  }

  public static Command intakeWithScaling() {
    // bs numbers please change them
    return Commands.startEnd(
        () ->
            intake.setGoalRPS(
                MathUtil.clamp(45 + 7.5 * drive.getSpeed(), 0.0, IntakeConstants.maxRPS)),
        () -> intake.setGoalRPS(0),
        intake);
  }

  public static Command setTurretOPAuto() {
    return Commands.runOnce(() -> turret.setGoal(0.581), turret);
  }

  // Corner Setpoints
  public static Command feederCornerAlign() {
    return Commands.parallel(
        Commands.run(() -> turret.setGoal(TurretConstants.rightCornerSetpoint), turret),
        Commands.run(() -> shooter.setVelocityRPS(ShooterConstants.shooterVelocityHubMap.get(5.2))),
        Commands.run(() -> hood.setGoal(ShooterHoodConstants.shooterHoodHubMap.get(5.2))));
  }

  public static Command depoCornerAlign() {
    return Commands.parallel(
        Commands.run(() -> turret.setGoal(TurretConstants.leftCornerSetpoint), turret),
        Commands.run(() -> shooter.setVelocityRPS(ShooterConstants.shooterVelocityHubMap.get(5.2))),
        Commands.run(() -> hood.setGoal(ShooterHoodConstants.shooterHoodHubMap.get(5.2))));
  }

  public static Command stopShooter() {
    return Commands.runOnce(() -> shooter.setVelocityRPS(0), shooter);
  }

  public static Command autonomousStopSOTM() {
    return Commands.parallel(
        stopShoot(),
        Commands.run(() -> shooter.setVelocityRPS(0)),
        Commands.run(() -> hood.setGoal(0)),
        Commands.run(() -> shooter.resetShooter()));
  }
}
