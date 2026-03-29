package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private static Indexer instance;

  public static Indexer getInstance() {
    return instance;
  }

  public static Indexer initialize(IndexerIO io) {
    if (instance == null) {
      instance = new Indexer(io);
    }
    return instance;
  }

  private Indexer(IndexerIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  public void setVoltageTower(double voltage) {
    io.setVoltageTower(voltage);
  }

  public void setVoltageIndexer(double voltage) {
    io.setVoltageIndexer(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateMotionProfile();
    Logger.processInputs("Indexer", inputs);
  }
}
