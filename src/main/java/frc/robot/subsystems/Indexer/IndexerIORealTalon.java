package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.util.LoggedTunableNumber;

public class IndexerIORealTalon implements IndexerIO {
  private final TalonFX towerMotor = new TalonFX(IndexerConstants.towerCanID);
  private final SparkFlex indexerMotor =
      new SparkFlex(IndexerConstants.indexerCanID, MotorType.kBrushless);
  private final RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private SparkFlexConfig config = new SparkFlexConfig();
  private TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private double towerVolts = 0.0;
  private double indexerVolts = 0.0;

  LoggedTunableNumber KP = new LoggedTunableNumber("Indexer/KP", IndexerConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("Indexer/KI", IndexerConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("Indexer/KD", IndexerConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("Indexer/KS", IndexerConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("Indexer/KV", IndexerConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("Indexer/KA", IndexerConstants.kA);

  SparkClosedLoopController controller;

  private double indexerRPS = 0.0;

  public IndexerIORealTalon() {
    // Tower Talon config
    fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    fxConfig.CurrentLimits.StatorCurrentLimit = 40;
    fxConfig.CurrentLimits.SupplyCurrentLimit = 40;

    fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Indexer SparkFlex config
    indexerMotor.clearFaults();
    config.smartCurrentLimit(50);
    config.idleMode(IdleMode.kCoast);

    config.closedLoop.feedForward.sva(KS.getAsDouble(), KV.getAsDouble(), KA.getAsDouble());
    config.closedLoop.pid(KP.getAsDouble(), KI.getAsDouble(), KD.getAsDouble());
    config.openLoopRampRate(0.0);

    indexerMotor.setCANTimeout(20);

    towerMotor.getConfigurator().apply(fxConfig);
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    controller = indexerMotor.getClosedLoopController();
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.towerVolts = this.towerVolts;
    inputs.towerAngularVelocityRPS = towerMotor.getVelocity().getValueAsDouble() / 60.0;
    inputs.towerCurrentAmps = towerMotor.getStatorCurrent().getValueAsDouble();

    inputs.indexerVolts = this.indexerVolts;
    inputs.indexerAngularVelocityRPS = indexerEncoder.getVelocity() / 60.0;
    inputs.indexerCurrentAmps = indexerMotor.getOutputCurrent();

    inputs.indexerGoalRPS = this.indexerRPS;
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

  @Override
  public void setRPSIndexer(double goalRPS) {
    if (indexerRPS != goalRPS) {
      indexerRPS = goalRPS;
    }
  }

  // @Override
  // public void updateMotionProfile() {
  //   controller.setSetpoint(
  //       indexerRPS * 60.0, ControlType.kVelocity); // do we want max motion instead?
  // }

  @Override
  public double getCurrentTower() {
    return towerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void configure() {
    config.closedLoop.feedForward.sva(KS.getAsDouble(), KV.getAsDouble(), KA.getAsDouble());
    config.closedLoop.pid(KP.getAsDouble(), KI.getAsDouble(), KD.getAsDouble());
    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
