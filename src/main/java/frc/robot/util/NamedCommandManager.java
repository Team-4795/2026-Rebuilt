package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
  public static void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", AutoCommands.shoot());
    NamedCommands.registerCommand("Align Hub", AutoCommands.autoScore());
    NamedCommands.registerCommand("Second Align", AutoCommands.autoScore());
    NamedCommands.registerCommand("Aim at Hub", AutoCommands.turretAimAtHub());
    NamedCommands.registerCommand("Intake", AutoCommands.intake());
    NamedCommands.registerCommand("Shooter Hood Angle", AutoCommands.setShooterHoodDynamic());
    NamedCommands.registerCommand("Testing Hood", AutoCommands.setHoodAngle(-0.05));
    NamedCommands.registerCommand("Retract Intake", AutoCommands.retractIntake());
    NamedCommands.registerCommand("Deploy Intake", AutoCommands.deployIntake());
    NamedCommands.registerCommand("Shoot on Move", AutoCommands.movingAlign());
    NamedCommands.registerCommand("Zero Hood Angle", AutoCommands.setHoodAngle(0));
  }
}
