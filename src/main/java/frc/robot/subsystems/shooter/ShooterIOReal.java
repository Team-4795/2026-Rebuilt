package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
  private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

  private final StatusSignal<AngularVelocity> topRPM = topShooterMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomRPM = bottomShooterMotor.getVelocity();
  private final StatusSignal<Current> topCurrent = bottomShooterMotor.getTorqueCurrent();
  private final StatusSignal<Current> bottomCurrent = bottomShooterMotor.getTorqueCurrent();

  private double topVolts = 0.0;
  private double bottomVolts = 0.0;

  // public VelocityVoltage controller = new VelocityVoltage();

  private TalonFXConfiguration config(double kV) {
    var talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Slot0.kP = ShooterConstants.kP;
    talonFXConfig.Slot0.kI = ShooterConstants.kI;
    talonFXConfig.Slot0.kD = ShooterConstants.kD;
    talonFXConfig.Slot0.kS = ShooterConstants.kS;
    talonFXConfig.Slot0.kV = ShooterConstants.kV;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Audio.BeepOnBoot = true;

    return talonFXConfig;
  }

  public ShooterIOReal() {
    var topConfig = config(ShooterConstants.kV);
    var bottomConfig = config(ShooterConstants.kV);

    BaseStatusSignal.setUpdateFrequencyForAll(50, topRPM, bottomRPM, topCurrent, bottomCurrent);

    topShooterMotor.optimizeBusUtilization(1.0);
    bottomShooterMotor.optimizeBusUtilization(1.0);

    bottomShooterMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();

    topShooterMotor.getConfigurator().apply(topConfig);
    bottomShooterMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void setVoltageTop(double volts) {
    topVolts = volts;
    topShooterMotor.setVoltage(volts);
  }

  @Override
  public void setVoltageBottom(double volts) {
    bottomVolts = volts;
    bottomShooterMotor.setVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(topRPM, bottomRPM, topCurrent, bottomCurrent);

    inputs.bottomShooterVelocityRPM = bottomRPM.getValueAsDouble() * 60.0; // RPS to RPM
    inputs.bottomShooterCurrent = bottomCurrent.getValueAsDouble();
    inputs.bottomShooterVolts = bottomVolts;

    inputs.topShooterVelocityRPM = topRPM.getValueAsDouble() * 60.0; // RPS to RPM
    inputs.topShooterCurrent = topCurrent.getValueAsDouble();
    inputs.topShooterVolts = topVolts;
  }
}
