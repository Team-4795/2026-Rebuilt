package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClimbIOReal implements ClimbIO {
  SparkFlex climb = new SparkFlex(ClimbConstants.canID, MotorType.kBrushless);
  RelativeEncoder encoder = climb.getEncoder();
  double voltage = 0;

  SparkFlexConfig config = new SparkFlexConfig();

  public ClimbIOReal() {
    config.smartCurrentLimit(ClimbConstants.current);
    config.idleMode(IdleMode.kBrake);
    climb.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climb.clearFaults();
  }

  @Override
  public void setVoltage(double volts) {
    voltage = volts;
    climb.setVoltage(volts);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.velocity = encoder.getVelocity();
    inputs.current = climb.getAppliedOutput();
    inputs.volts = voltage;
  }
}
