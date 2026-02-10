package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret.Turret;

public class ZeroSequence extends Command {
  // ADD SHOOTER HOOD!!!!
  private Turret turret;

  public ZeroSequence(Turret t) {
    turret = t;
  }

  @Override
  public void execute() {}
}
