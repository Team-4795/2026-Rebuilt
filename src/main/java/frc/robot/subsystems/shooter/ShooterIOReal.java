package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
  private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);

  private final StatusSignal<AngularVelocity> topRPM = topShooterMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomRPM = bottomShooterMotor.getVelocity();
  private final StatusSignal<Current> topCurrent = topShooterMotor.getTorqueCurrent();
  private final StatusSignal<Current> bottomCurrent = bottomShooterMotor.getTorqueCurrent();

  final MotionMagicVelocityTorqueCurrentFOC request = new MotionMagicVelocityTorqueCurrentFOC(0);
  final Follower follower =
      new Follower(
          ShooterConstants.TOP_CAN_ID, MotorAlignmentValue.Aligned); // double check alignment later

  private double velocityRPS = 0.0;

  private double volts = 0.0;

  public TalonFXConfiguration config(double kV) {
    var talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Slot0.kS = ShooterConstants.kS;
    talonFXConfig.Slot0.kV = kV;
    talonFXConfig.Slot0.kA = ShooterConstants.kA;
    talonFXConfig.Slot0.kP = ShooterConstants.kP;
    talonFXConfig.Slot0.kI = ShooterConstants.kI;
    talonFXConfig.Slot0.kD = ShooterConstants.kD;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.CURRENT_LIMIT;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Audio.BeepOnBoot = true;

    var motionMagicConfig = talonFXConfig.MotionMagic;

    motionMagicConfig.MotionMagicAcceleration = ShooterConstants.MM_ACCELERATION;
    motionMagicConfig.MotionMagicJerk = ShooterConstants.MM_JERK;

    return talonFXConfig;
  }

  public ShooterIOReal() {
    var topConfig = config(ShooterConstants.kV);
    var bottomConfig = config(ShooterConstants.kV);
    bottomConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    // set follower?

    BaseStatusSignal.setUpdateFrequencyForAll(50, topRPM, bottomRPM, topCurrent, bottomCurrent);

    topShooterMotor.optimizeBusUtilization(1.0);
    bottomShooterMotor.optimizeBusUtilization(1.0);

    bottomShooterMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();

    topShooterMotor.getConfigurator().apply(topConfig);
    bottomShooterMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    velocityRPS = velocityRPM / 60; // convert to rps
    topShooterMotor.setControl(request.withVelocity(velocityRPS));
    bottomShooterMotor.setControl(follower);
  }

  @Override
  public void setVoltage(double volts) {
    this.volts = volts;
    topShooterMotor.setVoltage(volts);
    bottomShooterMotor.setControl(follower);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(topRPM, bottomRPM, topCurrent, bottomCurrent);

    inputs.bottomShooterVelocityRPM = bottomRPM.getValueAsDouble() * 60.0; // RPS to RPM
    inputs.bottomShooterCurrent = bottomCurrent.getValueAsDouble();
    inputs.bottomShooterVolts = volts;

    inputs.topShooterVelocityRPM = topRPM.getValueAsDouble() * 60.0; // RPS to RPM
    inputs.topShooterCurrent = topCurrent.getValueAsDouble();
    inputs.topShooterVolts = volts;
  }
}
