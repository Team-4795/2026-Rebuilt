package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    this.io = turretIO;
    io.updateInputs(inputs);
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  public void resetTurret() {
    io.resetTurret();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("hub", Constants.hub);
  }
}
