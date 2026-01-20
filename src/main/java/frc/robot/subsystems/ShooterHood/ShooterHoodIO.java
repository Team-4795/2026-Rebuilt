package frc.robot.subsystems.ShooterHood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public double position = 0.0;
    public double velocityRPS = 0.0;
    public double current = 0.0;
  }

  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  public default double getPosition() {
    return 0.0;
  }

  public default void setGoal(double goal) {}
}
