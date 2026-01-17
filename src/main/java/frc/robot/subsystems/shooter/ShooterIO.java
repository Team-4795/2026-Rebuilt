package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    public double topShooterVelocityRPM = 0.0;
    public double topShooterCurrent = 0.0;
    public double topShooterVolts = 0.0;

    public double bottomShooterVelocityRPM = 0.0;
    public double bottomShooterCurrent = 0.0;
    public double bottomShooterVolts = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setVoltageTop(double volts) {}

  public default void setVoltageBottom(double volts) {}
}
