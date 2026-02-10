package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  DCMotorSim towerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.001, 30),
          DCMotor.getKrakenX44(1));

  private double volts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    towerMotor.update(0.02);

    inputs.voltage = this.volts;
    inputs.angularVelocityRPS = towerMotor.getAngularVelocityRPM() / 60.0;
    inputs.angularPositionRot = towerMotor.getAngularPositionRotations();
  }

  @Override
  public void setVoltage(double voltage) {
    towerMotor.setInputVoltage(voltage);
    this.volts = voltage;
  }
}
