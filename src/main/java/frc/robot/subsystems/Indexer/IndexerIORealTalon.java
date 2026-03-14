package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IndexerIORealTalon implements IndexerIO {
  private final TalonFX towerMotor =
      new TalonFX(IndexerConstants.towerCanID);
  private final SparkFlex indexerMotor =
      new SparkFlex(IndexerConstants.indexerCanID, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private SparkFlexConfig config = new SparkFlexConfig();
  private TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private double towerVolts = 0.0;
  private double indexerVolts = 0.0;

  public IndexerIORealTalon() {
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    fxConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.currentLimit;
    fxConfig.CurrentLimits.SupplyCurrentLimit = IndexerConstants.currentLimit;

    fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    indexerMotor.clearFaults();

    config.smartCurrentLimit(IndexerConstants.currentLimit);
    config.idleMode(IdleMode.kCoast);

    indexerMotor.setCANTimeout(20);

    towerMotor.getConfigurator().apply(fxConfig);
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.towerVolts = this.towerVolts;
    inputs.towerAngularVelocityRPS = towerMotor.getVelocity().getValueAsDouble() / 60.0;
    inputs.towerCurrentAmps = towerMotor.getStatorCurrent().getValueAsDouble();

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
