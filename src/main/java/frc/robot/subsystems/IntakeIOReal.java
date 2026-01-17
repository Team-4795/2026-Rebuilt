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

  private TalonFXConfiguration motorAConfig = new TalonFXConfiguration();
  private TalonFXConfiguration motorBConfig;

  private final StatusSignal<Current> currentA = intakeMotorA.getStatorCurrent();
  private final StatusSignal<Voltage> voltageA = intakeMotorA.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocityA = intakeMotorA.getVelocity();

  private final StatusSignal<Current> currentB = intakeMotorA.getStatorCurrent();
  private final StatusSignal<Voltage> voltageB = intakeMotorA.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocityB = intakeMotorA.getVelocity();

  public IntakeIOReal() {
    motorAConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorAConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.CURRENT_LIMIT;
    motorAConfig.Audio.BeepOnBoot = true;
    motorAConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // same configs, but inverted
    motorBConfig = motorAConfig.clone();
    motorBConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

    // apply configs and check response
    StatusCode responseA = intakeMotorA.getConfigurator().apply(motorAConfig);
    StatusCode responseB = intakeMotorB.getConfigurator().apply(motorBConfig);

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
  }

  @Override
  public void setSpeed(double speed) {
    intakeMotorA.set(speed);
    intakeMotorB.set(speed);
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
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityA, voltageA, currentA);
    BaseStatusSignal.refreshAll(velocityB, voltageB, currentB);

    inputs.angularVelocityRPMA = velocityA.getValueAsDouble() * 60;
    inputs.angularPositionRotA = intakeMotorA.getPosition().getValueAsDouble();
    inputs.currentAmpsA = currentA.getValueAsDouble();
    inputs.voltageA = voltageA.getValueAsDouble();

    inputs.angularVelocityRPMB = velocityB.getValueAsDouble() * 60;
    inputs.angularPositionRotB = intakeMotorB.getPosition().getValueAsDouble();
    inputs.currentAmpsB = currentB.getValueAsDouble();
    inputs.voltageB = voltageB.getValueAsDouble();
  }
}
