package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double goal = 0;
    public double position = 0;
    public double current = 0;
    public double volts = 0;
    public double velocity = 0;
  }

  public default double getPosition() {
    return 0;
  }

  public default void setVoltage(double voltage) {}

  public default void setGoal(double goal) {}

  public default void updateInputs(TurretIOInputs inputs) {}
}
