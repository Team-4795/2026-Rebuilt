package frc.robot.subsystems.ShooterHood;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterHoodIO {
  @AutoLog
  public static class ShooterHoodIOInputs {
    public double goalRotations = 0.0;
    public double position = 0.0;
    public double velocityRPS = 0.0;
    public double current = 0.0;
    public double voltage = 0.0;
  }

  public default void updateInputs(ShooterHoodIOInputs inputs) {}

  public default double getPosition() {
    return 0.0;
  }

  public default double getGoal() {
    return 0.0;
  }

  public default void setVoltage(double volts) {}

  public default void setGoal(double goal) {}

  public default void zero() {}

  public default boolean readyToShoot() {
    return false;
  }
  public default void configure() {
    
  }
}
