package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
  private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

  private final StatusSignal<AngularVelocity> topRPS = topShooterMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomRPS = bottomShooterMotor.getVelocity();
  private final StatusSignal<Current> topCurrent = topShooterMotor.getTorqueCurrent();
  private final StatusSignal<Current> bottomCurrent = bottomShooterMotor.getTorqueCurrent();

  LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/KP", ShooterConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("Shooter/KI", ShooterConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/KD", ShooterConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("Shooter/KS", ShooterConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("Shooter/KV", ShooterConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("Shooter/KA", ShooterConstants.kA);

  private final MotionMagicVelocityTorqueCurrentFOC m_request =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  private double volts = 0.0;
  private double goalVelocityRPS = 0.0;

  public ShooterIOReal() {
    var topConfig = config(ShooterConstants.kV);
    var bottomConfig = config(ShooterConstants.kV);

    BaseStatusSignal.setUpdateFrequencyForAll(50, topRPS, bottomRPS, topCurrent, bottomCurrent);

    topShooterMotor.optimizeBusUtilization(1.0);
    bottomShooterMotor.optimizeBusUtilization(1.0);

    bottomShooterMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();

    // Bottom motor follows top motor. Spin in opposite directions
    bottomShooterMotor.setControl(
        new Follower(topShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    topShooterMotor.getConfigurator().apply(topConfig);
    bottomShooterMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void setVelocityRPS(double velocityRPS) {
    velocityRPS = MathUtil.clamp(velocityRPS, ShooterConstants.minVel, ShooterConstants.maxVel);
    topShooterMotor.setControl(m_request.withVelocity(velocityRPS));
    this.goalVelocityRPS = velocityRPS;
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    topShooterMotor.setVoltage(volts);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(topRPS, bottomRPS, topCurrent, bottomCurrent);

    inputs.goalVelocityRPS = goalVelocityRPS;

    inputs.bottomShooterVelocityRPS = bottomRPS.getValueAsDouble();
    inputs.bottomShooterCurrent = bottomCurrent.getValueAsDouble();
    inputs.bottomShooterVolts = volts;

    inputs.topShooterVelocityRPS = topRPS.getValueAsDouble();
    inputs.topShooterCurrent = topCurrent.getValueAsDouble();
    inputs.topShooterVolts = volts;
  }

  public TalonFXConfiguration config(double kV) {
    var talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Audio.BeepOnBoot = true;

    // PID + FF settings
    talonFXConfig.Slot0.kS = ShooterConstants.kS;
    talonFXConfig.Slot0.kV = kV;
    talonFXConfig.Slot0.kA = ShooterConstants.kA;
    talonFXConfig.Slot0.kP = ShooterConstants.kP;
    talonFXConfig.Slot0.kI = ShooterConstants.kI;
    talonFXConfig.Slot0.kD = ShooterConstants.kD;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;

    talonFXConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEARING;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Motion Magic settings
    var motionMagicConfig = talonFXConfig.MotionMagic;
    motionMagicConfig.MotionMagicAcceleration = ShooterConstants.MM_ACCELERATION;
    motionMagicConfig.MotionMagicJerk = ShooterConstants.MM_JERK;

    return talonFXConfig;
  }
}
