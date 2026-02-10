package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {

    public double goalVelocityRPS = 0.0;

    public double topShooterVelocityRPS = 0.0;
    public double topShooterCurrent = 0.0;
    public double topShooterVolts = 0.0;

    public double bottomShooterVelocityRPS = 0.0;
    public double bottomShooterCurrent = 0.0;
    public double bottomShooterVolts = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default double getTopRPS() {
    return 0.0;
  }

  public default double getBottomRPS() {
    return 0.0;
  }
  
  public default double getGoal() {
    return 0.0;
  }

  public default void setVelocityRPS(double velocityRPS) {}

  public default void setVoltage(double volts) {}

  public default boolean readyToShoot() {
    return false;
  }

}
