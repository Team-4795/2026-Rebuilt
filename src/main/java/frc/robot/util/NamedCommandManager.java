package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
  public static void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", AutoCommands.shoot());
    NamedCommands.registerCommand("Align Hub", AutoCommands.autoScore());
    NamedCommands.registerCommand("Second Align", AutoCommands.autoScore());
    NamedCommands.registerCommand("Aim at Hub", AutoCommands.turretAimAtHub());
    NamedCommands.registerCommand("Intake", AutoCommands.intake().withTimeout(3));
    NamedCommands.registerCommand("Shooter Hood Angle", AutoCommands.setShooterHoodDynamic());
  }
}
