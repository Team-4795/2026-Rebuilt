package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double voltageA = 0.0;
    public double angularPositionRotA = 0.0;
    public double angularVelocityRPMA = 0.0;
    public double currentAmpsA = 0.0;

    public double voltageB = 0.0;
    public double angularPositionRotB = 0.0;
    public double angularVelocityRPMB = 0.0;
    public double currentAmpsB = 0.0;
  }

  public default void setSpeed(double speed) {}

  public default double getSpeed() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  public default double getCurrent() {
    return 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
