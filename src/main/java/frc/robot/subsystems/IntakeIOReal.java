package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
  // two motors on each side of the spinny bar thing
  private final TalonFX intakeMotorA = new TalonFX(IntakeConstants.CAN_ID_A);
  private final TalonFX intakeMotorB = new TalonFX(IntakeConstants.CAN_ID_B);
  // one motor to extend intake
  private final TalonFX intakeDeployMotor = new TalonFX(IntakeConstants.CAN_ID_DEPLOY);

  private TalonFXConfiguration motorAConfig = config(0);
  private TalonFXConfiguration motorBConfig = config(0);
  private TalonFXConfiguration deployMotorConfig = config(1);

  private final StatusSignal<Current> currentA = intakeMotorA.getStatorCurrent();
  private final StatusSignal<Voltage> voltageA = intakeMotorA.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocityA = intakeMotorA.getVelocity();

  private final StatusSignal<Current> currentB = intakeMotorA.getStatorCurrent();
  private final StatusSignal<Voltage> voltageB = intakeMotorA.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocityB = intakeMotorA.getVelocity();

  private final StatusSignal<Current> currentDeploy = intakeDeployMotor.getStatorCurrent();
  private final StatusSignal<Voltage> voltageDeploy = intakeDeployMotor.getMotorVoltage();

  public IntakeIOReal() {
    // same configs as other motors, but inverted
    motorBConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    intakeMotorA.clearStickyFaults();
    intakeMotorB.clearStickyFaults();
    intakeDeployMotor.clearStickyFaults();

    // Use if needed
    // BaseStatusSignal
    //   .setUpdateFrequencyForAll(50,
    //     currentA, voltageA, velocityA,
    //     currentB, voltageB, velocityB);
    //
    // intakeMotorA.optimizeBusUtilization(1.0);
    // intakeMotorB.optimizeBusUtilization(1.0);

    // apply configs and check response
    StatusCode responseA = intakeMotorA.getConfigurator().apply(motorAConfig);
    StatusCode responseB = intakeMotorB.getConfigurator().apply(motorBConfig);
    StatusCode responseDeploy = intakeDeployMotor.getConfigurator().apply(deployMotorConfig);

    if (!responseA.isOK()) {
      System.out.println(
          "Talon ID "
              + intakeMotorA.getDeviceID()
              + " failed config with error "
              + responseA.toString());
    }

    if (!responseB.isOK()) {
      System.out.println(
          "Talon ID "
              + intakeMotorB.getDeviceID()
              + " failed config with error "
              + responseB.toString());
    }

    if (!responseDeploy.isOK()) {
      System.out.println(
          "Talon ID "
              + intakeMotorB.getDeviceID()
              + " failed config with error "
              + responseB.toString());
    }
  }

  @Override
  public void setDeployVoltage(double v) {
    intakeDeployMotor.setVoltage(v);
  }

  @Override
  public void setVoltage(double v) {
    intakeMotorA.setVoltage(v);
    intakeMotorB.setVoltage(v);
  }

  // the motors should be spinning at the same speed/voltage
  @Override
  public double getSpeed() {
    return velocityA.getValueAsDouble() * 60; // rpm
  }

  @Override
  public double getVoltage() {
    return voltageA.getValueAsDouble();
  }

  @Override
  public double getCurrent() {
    return currentA.getValueAsDouble();
  }

  @Override
  public double getDeployVoltage() {
    return voltageDeploy.getValueAsDouble();
  }

  @Override
  public double getDeployCurrent() {
    return currentDeploy.getValueAsDouble();
  }

  // type 0 is regular intake, type 1 is deploy motor
  private TalonFXConfiguration config(int type) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    if (type == 0) config.Feedback.SensorToMechanismRatio = IntakeConstants.GEARING;
    else config.Feedback.SensorToMechanismRatio = IntakeConstants.GEARING_DEPLOY;

    config.Audio.BeepOnBoot = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    return config;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityA, voltageA, currentA);
    BaseStatusSignal.refreshAll(velocityB, voltageB, currentB);
    BaseStatusSignal.refreshAll(voltageDeploy, currentDeploy);

    inputs.angularVelocityRPMA = velocityA.getValueAsDouble() * 60;
    inputs.angularPositionRotA = intakeMotorA.getPosition().getValueAsDouble();
    inputs.currentAmpsA = currentA.getValueAsDouble();
    inputs.voltageA = voltageA.getValueAsDouble();

    inputs.angularVelocityRPMB = velocityB.getValueAsDouble() * 60;
    inputs.angularPositionRotB = intakeMotorB.getPosition().getValueAsDouble();
    inputs.currentAmpsB = currentB.getValueAsDouble();
    inputs.voltageB = voltageB.getValueAsDouble();

    inputs.currentAmpsDeploy = currentDeploy.getValueAsDouble();
    inputs.voltageDeploy = voltageDeploy.getValueAsDouble();
  }
}
