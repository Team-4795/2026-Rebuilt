package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IndexerIOReal implements IndexerIO {
  private final SparkFlex towerMotor = new SparkFlex(IndexerConstants.canID, MotorType.kBrushless);
  private final RelativeEncoder encoder = towerMotor.getEncoder();

  private SparkFlexConfig config = new SparkFlexConfig();
  private double volts = 0.0;

  public IndexerIOReal() {
    towerMotor.clearFaults();
    config.smartCurrentLimit(IndexerConstants.currentLimit);
    config.idleMode(IdleMode.kCoast);
    towerMotor.setCANTimeout(20);
    towerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.voltage = this.volts;
    inputs.angularVelocityRPS = encoder.getVelocity() / 60.0;
    inputs.angularPositionRot = encoder.getPosition();
    inputs.currentAmps = towerMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    this.volts = voltage;
    towerMotor.setVoltage(voltage);
  }
}
