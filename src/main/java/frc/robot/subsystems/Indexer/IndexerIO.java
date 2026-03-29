package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double towerVolts = 0.0;
    public double towerAngularVelocityRPS = 0.0;
    public double towerCurrentAmps = 0.0;

    public double indexerVolts = 0.0;
    public double indexerAngularVelocityRPS = 0.0;
    public double indexerCurrentAmps = 0.0;

    public double indexerGoalRPS = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltageTower(double voltage) {}

  public default void setVoltageIndexer(double voltage) {}

  public default void setGoal(double goalRPS) {}

  public default void updateMotionProfile() {}

  public default void configure() {}
}
