package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private ClimbIO io;
  private ClimbIOAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private static Climb instance;

  public static Climb initialize(ClimbIO io) {
    if (instance == null) {
      instance = new Climb(io);
    }
    return instance;
  }

  public static Climb getInstance() {
    return instance;
  }

  private Climb(ClimbIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public void setIntakeVoltage(double volts) {
    io.setVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
