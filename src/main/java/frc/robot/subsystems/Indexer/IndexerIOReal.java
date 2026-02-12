package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IndexerIOReal implements IndexerIO {
  private final SparkFlex towerMotor =
      new SparkFlex(IndexerConstants.towerCanID, MotorType.kBrushless);
  private final SparkFlex indexerMotor =
      new SparkFlex(IndexerConstants.indexerCanID, MotorType.kBrushless);
  private final RelativeEncoder towerEncoder = towerMotor.getEncoder();
  private final RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private SparkFlexConfig config = new SparkFlexConfig();

  private double towerVolts = 0.0;
  private double indexerVolts = 0.0;

  public IndexerIOReal() {
    towerMotor.clearFaults();
    indexerMotor.clearFaults();

    config.smartCurrentLimit(IndexerConstants.currentLimit);
    config.idleMode(IdleMode.kCoast);

    towerMotor.setCANTimeout(20);
    indexerMotor.setCANTimeout(20);

    towerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.towerVolts = this.towerVolts;
    inputs.towerAngularVelocityRPS = towerEncoder.getVelocity() / 60.0;
    inputs.towerCurrentAmps = towerMotor.getOutputCurrent();

    inputs.indexerVolts = this.indexerVolts;
    inputs.indexerAngularVelocityRPS = indexerEncoder.getVelocity() / 60.0;
    inputs.indexerCurrentAmps = indexerMotor.getOutputCurrent();
  }

  @Override
  public void setVoltageTower(double voltage) {
    this.towerVolts = voltage;
    towerMotor.setVoltage(voltage);
  }

  @Override
  public void setVoltageIndexer(double voltage) {
    this.indexerVolts = voltage;
    indexerMotor.setVoltage(voltage);
  }
}
