package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private static Turret instance;

  public static Turret getInstance() {
    return instance;
  }

  public static Turret initialize(TurretIO io) {
    if (instance == null) {
      instance = new Turret(io);
    }
    return instance;
  }

  private Turret(TurretIO turretIO) {
    io = turretIO;
    io.updateInputs(inputs);
  }

  public Command setGoal(double goal) {
    return Commands.runOnce(() -> io.setGoal(goal), this);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }
}
