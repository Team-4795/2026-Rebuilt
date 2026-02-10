package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double voltage = 0.0;
    public double angularPositionRot = 0.0;
    public double angularVelocityRPS = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
