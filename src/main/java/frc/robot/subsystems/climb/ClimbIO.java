package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    double volts = 0;
    double current = 0;
    double velocity = 0;
  }

  public default void setVoltage(double volts) {}

  public default void updateInputs(ClimbIOInputs inputs) {}
}
