package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double voltageA = 0.0;
    public double angularPositionRotA = 0.0;
    public double angularVelocityRPSA = 0.0;
    public double currentAmpsA = 0.0;

    public double voltageB = 0.0;
    public double angularPositionRotB = 0.0;
    public double angularVelocityRPSB = 0.0;
    public double currentAmpsB = 0.0;

    public double voltageDeploy = 0.0;
    public double currentAmpsDeploy = 0.0;
  }

  public default void setDeployVoltage(double volts) {}

  public default void setVoltage(double volts) {}

  public default double getSpeed() {
    return 0;
  }

  public default double getVoltage() {
    return 0;
  }

  public default double getCurrent() {
    return 0;
  }

  public default double getDeployPos() {
    return 0;
  }

  public default double getDeployVoltage() {
    return 0;
  }

  public default double getDeployCurrent() {
    return 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
