package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  public double getPosition() {
    return inputs.angularPositionRot;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
