package frc.robot.subsystems.IntakeDeploy;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public double deployMotorVoltage = 0.0;

    public double deployMotorPositionA = 0.0;
    public double deployMotorVelocityA = 0.0;

    public double deployMotorVelocityB = 0.0;
    public double deployMotorPositionB = 0.0;

    public double deployMotorGoal = 0.0;
    public double deployMotorSetpoint = 0.0;

    public double setpointPosition = 0;
    public double setpointVelocity = 0;
  }

  public default void setGoal(double goal) {}

  public default void updateMotionProfile() {}

  public default void setVoltage(double volts) {}

  public default void zero() {}

  public default double getPosition() {
    return 0.0;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}
}
