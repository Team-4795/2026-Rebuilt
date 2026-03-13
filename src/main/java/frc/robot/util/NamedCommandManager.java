package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
  public static void registerNamedCommands() {
    NamedCommands.registerCommand("Shoot", AutoCommands.shoot());
    NamedCommands.registerCommand("Align Hub", AutoCommands.autoScore());
    NamedCommands.registerCommand("Second Align", AutoCommands.autoScore());
    NamedCommands.registerCommand("Intake", AutoCommands.intake());
    NamedCommands.registerCommand(
        "Shooter Hood Angle", AutoCommands.setShooterHoodDynamic().withTimeout(100));
    NamedCommands.registerCommand("Testing Hood", AutoCommands.setHoodAngle(-0.05));
    NamedCommands.registerCommand("Retract Intake", AutoCommands.retractIntake());
    NamedCommands.registerCommand("Deploy Intake", AutoCommands.deployIntake());
    NamedCommands.registerCommand("Zero Hood Angle", AutoCommands.setHoodAngle(0));
    NamedCommands.registerCommand("Everything", AutoCommands.pleaseNoTouch());
  }
}
