package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.CAN_ID);
  private TalonFXSimState simMotor;

  private TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  private MotionMagicVoltage control = new MotionMagicVoltage(0);
  private MotionMagicConfigs controlConfig = new MotionMagicConfigs();

  private final DCMotorSim m_motorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.02, 60),
          DCMotor.getKrakenX44(1));

  public TurretIOSim() {
    turretConfig.Slot0.kA = TurretConstants.kA;
    turretConfig.Slot0.kV = TurretConstants.simkV;
    turretConfig.Slot0.kS = TurretConstants.kS;
    turretConfig.Slot0.kP = TurretConstants.simkP;
    turretConfig.Slot0.kI = TurretConstants.kI;
    turretConfig.Slot0.kD = TurretConstants.kD;

    turretConfig.CurrentLimits.StatorCurrentLimit = 60;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretConfig.Feedback.SensorToMechanismRatio = TurretConstants.gearing;

    turretMotor.getConfigurator().apply(turretConfig);

    controlConfig.MotionMagicAcceleration = TurretConstants.maxAcceleration;
    controlConfig.MotionMagicCruiseVelocity = TurretConstants.maxVelocity;
    controlConfig.MotionMagicJerk = TurretConstants.maxJerk;

    turretMotor.getConfigurator().apply(controlConfig);
  }

  @Override
  public void setGoal(double goal) {
    turretMotor.setControl(control.withPosition(goal));
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    simMotor = turretMotor.getSimState();
    simMotor.setSupplyVoltage(12.0);

    m_motorSimModel.setInputVoltage(simMotor.getMotorVoltage());
    m_motorSimModel.update(0.020);

    double rotorPos = m_motorSimModel.getAngularPositionRotations() * TurretConstants.gearing;
    double rotorVelRPS = m_motorSimModel.getAngularVelocityRPM() * TurretConstants.gearing / 60.0;

    simMotor.setRawRotorPosition(rotorPos);
    simMotor.setRotorVelocity(rotorVelRPS);

    inputs.goal = turretMotor.getClosedLoopReference().getValueAsDouble();
    inputs.position = m_motorSimModel.getAngularPositionRotations();
    inputs.velocity = m_motorSimModel.getAngularVelocityRPM() / 60.0;
    inputs.current = m_motorSimModel.getCurrentDrawAmps();
    inputs.volts = m_motorSimModel.getInputVoltage();
  }
}
