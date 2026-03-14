package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.LoggedTunableNumber;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
  private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

  private TalonFXConfiguration topConfig = new TalonFXConfiguration();
  private TalonFXConfiguration bottomConfig = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> topRPS = topShooterMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomRPS = bottomShooterMotor.getVelocity();
  private final StatusSignal<Current> topCurrent = topShooterMotor.getTorqueCurrent();
  private final StatusSignal<Current> bottomCurrent = bottomShooterMotor.getTorqueCurrent();

  LoggedTunableNumber KP = new LoggedTunableNumber("Shooter/KP", ShooterConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("Shooter/KI", ShooterConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("Shooter/KD", ShooterConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("Shooter/KS", ShooterConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("Shooter/KV_TOP", ShooterConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("Shooter/KA", ShooterConstants.kA);

  LoggedTunableNumber MM_ACCELERATION =
      new LoggedTunableNumber("Shooter/Acceleration", ShooterConstants.MM_ACCELERATION);
  LoggedTunableNumber MM_JERK = new LoggedTunableNumber("Shooter/Jerk", ShooterConstants.MM_JERK);

  public static LoggedTunableNumber shooterRPS =
      new LoggedTunableNumber("Auto Shoot/Shooter RPS", 60);

  private final VelocityTorqueCurrentFOC m_request = new VelocityTorqueCurrentFOC(0);

  private double volts = 0.0;
  private double goalVelocityRPS = 0.0;

  public ShooterIOReal() {
    topConfig = config(ShooterConstants.kV);
    bottomConfig = config(ShooterConstants.kV);

    BaseStatusSignal.setUpdateFrequencyForAll(20, topRPS, bottomRPS, topCurrent, bottomCurrent);

    bottomShooterMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();

    topShooterMotor.getConfigurator().apply(topConfig);
    bottomShooterMotor.getConfigurator().apply(bottomConfig);

    bottomShooterMotor.setControl(
        new Follower(topShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public double getTopRPS() {
    return topRPS.getValueAsDouble();
  }

  @Override
  public double getBottomRPS() {
    return bottomRPS.getValueAsDouble();
  }

  @Override
  public double getGoal() {
    return this.goalVelocityRPS;
  }

  @Override
  public void setVelocityRPS(double velocityRPS) {
    velocityRPS = MathUtil.clamp(velocityRPS, ShooterConstants.minVel, ShooterConstants.maxVel);
    this.goalVelocityRPS = velocityRPS;

    if (this.goalVelocityRPS == 0) {
      topShooterMotor.setControl(new NeutralOut());
    } else {
      topShooterMotor.setControl(m_request.withVelocity(velocityRPS));
    }
  }

  @Override
  public void configure() {
    topConfig = config(KV.get());
    bottomConfig = config(KV.get());

    bottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    topConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    topShooterMotor.getConfigurator().apply(topConfig);
    bottomShooterMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    topShooterMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public boolean readyToShoot() {
    return (getGoal() != 0)
        && (getTopRPS() > (getGoal() - 6))
        && (getBottomRPS() > (getGoal() - 6));
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
    talonFXConfig.Slot0.kS = KS.get();
    talonFXConfig.Slot0.kV = kV;
    talonFXConfig.Slot0.kA = KA.get();
    talonFXConfig.Slot0.kP = KP.get();
    talonFXConfig.Slot0.kI = KI.get();
    talonFXConfig.Slot0.kD = KD.get();

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;
    talonFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT;

    talonFXConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEARING;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 120;
    talonFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

    // // Motion Magic settings
    // talonFXConfig.MotionMagic.MotionMagicAcceleration = MM_ACCELERATION.get();
    // talonFXConfig.MotionMagic.MotionMagicJerk = MM_JERK.get();

    return talonFXConfig;
  }
}
