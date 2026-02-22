package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class AutoCommands {
  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Indexer indexer = Indexer.getInstance();
  private static Turret turret = Turret.getInstance();
  private static ShooterHood hood = ShooterHood.getInstance();
  private static Intake intake = Intake.getInstance();
  private static IntakeDeploy deploy = IntakeDeploy.getInstance();

  public static LoggedTunableNumber hoodAngle =
      new LoggedTunableNumber("Auto Shoot/Hood Angle", -0.04);
  public static LoggedTunableNumber shooterRPS =
      new LoggedTunableNumber("Auto Shoot/Shooter RPS", 50);

  private AutoCommands() {}

  public static Command retractIntake() {
    return Commands.run(() -> deploy.setGoal(0), deploy);
  }

  public static Command intake() {
    return Commands.run(() -> intake.setIntakeVoltage(12), intake);
  }

  public static Command reverseIntake() {
    return Commands.run(() -> intake.setIntakeVoltage(-12), intake);
  }

  public static Command autoScore() {
    return Commands.parallel(
        turretAimAtTarget(), setHoodAngle(hoodAngle.get()), setShooterRPS(shooterRPS.get()));
  }

  public static Command shoot() {
    return Commands.parallel(
        Commands.run(() -> indexer.setVoltageIndexer(-12)),
        Commands.run(() -> indexer.setVoltageTower(-12)));
  }

  public static Command stopShoot() {
    return Commands.parallel(
        Commands.run(() -> indexer.setVoltageIndexer(0)),
        Commands.run(() -> indexer.setVoltageTower(0)));
  }

  public static Command zeroSequence() { // if it doesn't work check the motor limits
    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> turret.setVoltage(-3), turret),
            Commands.waitSeconds(1), // change
            Commands.runOnce(() -> turret.zero(), turret),
            Commands.runOnce(() -> turret.setVoltage(0), turret)),
        Commands.sequence(
            Commands.runOnce(() -> hood.setVoltage(4), hood),
            Commands.waitSeconds(1), // change
            Commands.runOnce(() -> hood.zero(), hood),
            Commands.runOnce(() -> hood.setVoltage(0), hood)));
  }

  public static Command turretAimAtHub() {
    return new AimAtHub(drive, turret);
  }

  public static Command turretAimAtTarget() {
    return new AimAtTarget(drive, turret, StateManager.getInstance());
  }

  public static Command underTrenchAssist() {
    return new UnderTrench();
  }

  public static Command setShooterRPS(double RPS) {
    return Commands.startEnd(
        () -> shooter.setVelocityRPS(RPS), () -> shooter.setVelocityRPS(0), shooter);
  }

  public static Command setHoodAngle(double angle) {
    return Commands.run(() -> hood.setGoal(angle), hood);
  }

  // Rev shooter wheels based on interpolation tree
  public static Command setShooterVelocityDynamic() {
    return Commands.startEnd(() -> shooter.setRPSDynamic(), () -> shooter.setVelocityRPS(0));
  }

  // Angle shooter hood based on interpolation tree
  public static Command setShooterHoodDynamic() {
    return Commands.run(() -> hood.setGoalDynamic());
  }
}
