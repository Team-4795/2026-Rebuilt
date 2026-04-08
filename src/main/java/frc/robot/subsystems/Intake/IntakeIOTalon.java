package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalon implements IntakeIO {
  TalonFX motor1 = new TalonFX(62);
  TalonFX motor2 = new TalonFX(61);

  TalonFXConfiguration config = new TalonFXConfiguration();

  public IntakeIOTalon() {
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motor1.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor2.getConfigurator().apply(config);
  }

  @Override
  public void setIntakeVoltage(double volts) {
    // motor1.setVoltage(volts);
    // motor2.setVoltage(volts);

    motor1.setControl(new VoltageOut(volts).withEnableFOC(true));
    motor2.setControl(new VoltageOut(volts).withEnableFOC(true));
  }
}
