package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  DCMotorSim towerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, 30),
          DCMotor.getNeoVortex(1));

  DCMotorSim indexerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, 30),
          DCMotor.getNeoVortex(1));

  private double towerVolts = 0.0;
  private double indexerVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    towerMotor.update(0.02);

    inputs.towerVolts = this.towerVolts;
    inputs.towerAngularVelocityRPS = towerMotor.getAngularVelocityRPM() / 60.0;

    inputs.indexerVolts = this.indexerVolts;
    inputs.indexerAngularVelocityRPS = indexerMotor.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void setVoltageTower(double voltage) {
    towerMotor.setInputVoltage(voltage);
    this.towerVolts = voltage;
  }

  @Override
  public void setVoltageIndexer(double voltage) {
    indexerMotor.setInputVoltage(voltage);
    this.indexerVolts = voltage;
  }
}
