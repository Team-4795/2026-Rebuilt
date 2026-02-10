package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
  private static Drive drive = Drive.getInstance();
  private static Shooter shooter = Shooter.getInstance();
  private static Indexer indexer = Indexer.getInstance();
  private static Turret turret = Turret.getInstance();
  private static ShooterHood hood = ShooterHood.getInstance();

  private AutoCommands() {}

  public static Command shoot() {
    return Commands.sequence(
        // indexer
        );
  }

  public static Command zeroSequence() { // if it doesn't work check the motor limits
    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> turret.setVoltage(-2), turret),
            Commands.waitSeconds(1), // change
            Commands.runOnce(() -> turret.zero(), turret),
            Commands.runOnce(() -> turret.setVoltage(0), turret)),
        Commands.sequence(
            Commands.runOnce(() -> hood.setVoltage(-2), hood),
            Commands.waitSeconds(1), // change
            Commands.runOnce(() -> hood.zero(), hood),
            Commands.runOnce(() -> hood.setVoltage(0), hood)));
  }
}
