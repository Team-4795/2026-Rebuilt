package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double voltageA = 0.0;
    public double angularVelocityRPSA = 0.0;
    public double currentAmpsA = 0.0;

    public double voltageB = 0.0;
    public double angularVelocityRPSB = 0.0;
    public double currentAmpsB = 0.0;

    public double deployMotorVoltage = 0.0;
    public double deployMotorPositionA = 0.0;
    public double deployMotorVelocityA = 0.0;
    public double deployMotorVelocityB = 0.0;
    public double deployMotorPositionB = 0.0;
    public double deployMotorGoal = 0.0;
    public double deployMotorSetpoint = 0.0;
  }

  public default void setGoal(double goal) {}

  public default void updateMotionProfile() {}

  public default void setDeployVoltage(double volts) {}

  public default void setIntakeVoltage(double volts) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
