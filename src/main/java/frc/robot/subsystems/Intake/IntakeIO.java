package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double voltageA = 0.0;
    public double angularVelocityRPSA = 0.0;
    public double currentAmpsA = 0.0;
    public double supplyCurrentA = 0;

    public double voltageB = 0.0;
    public double angularVelocityRPSB = 0.0;
    public double currentAmpsB = 0.0;
    public double supplyCurrentB = 0;

    public double goalRPS = 0.0;

    public boolean hasCurrentLimitChanged = false;
  }

  public default void setIntakeVoltage(double volts) {}

  public default void setGoalRPS(double rps) {}

  public default void configure() {}

  public default void setTeleopCurrentLimits() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
