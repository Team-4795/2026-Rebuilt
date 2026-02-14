package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  DCMotorSim motorA =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.001, IntakeConstants.GEARING),
          DCMotor.getKrakenX44(1));

  DCMotorSim motorB =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.001, IntakeConstants.GEARING),
          DCMotor.getKrakenX44(1));

  DCMotorSim motorDeploy =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.001, IntakeConstants.GEARING_DEPLOY),
          DCMotor.getKrakenX60(1));

  @Override
  public void setIntakeVoltage(double v) {
    motorA.setInputVoltage(v);
    motorB.setInputVoltage(v);
  }

  @Override
  public void setDeployVoltage(double v) {
    motorDeploy.setInputVoltage(v);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motorA.update(0.02);
    motorB.update(0.02);
    motorDeploy.update(0.02);

    inputs.angularVelocityRPSA = motorA.getAngularVelocityRPM() / 60.0;
    inputs.currentAmpsA = motorA.getCurrentDrawAmps();
    inputs.voltageA = motorA.getInputVoltage();

    inputs.angularVelocityRPSB = motorB.getAngularVelocityRPM() / 60.0;
    inputs.currentAmpsB = motorB.getCurrentDrawAmps();
    inputs.voltageB = motorB.getInputVoltage();

    inputs.deployMotorVoltage = motorDeploy.getInputVoltage();
  }
}
