package frc.robot.subsystems.ShooterHood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class ShooterHoodIOReal implements ShooterHoodIO {
  private TalonFX shooterHoodMotor = new TalonFX(ShooterHoodConstants.CAN_ID);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> position = shooterHoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRPS = shooterHoodMotor.getVelocity();
  private final StatusSignal<Current> current = shooterHoodMotor.getTorqueCurrent();
  private final StatusSignal<Voltage> voltage = shooterHoodMotor.getMotorVoltage();

  LoggedTunableNumber KP = new LoggedTunableNumber("ShooterHood/KP", ShooterHoodConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("ShooterHood/KI", ShooterHoodConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("ShooterHood/KD", ShooterHoodConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("ShooterHood/KS", ShooterHoodConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("ShooterHood/KV", ShooterHoodConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("ShooterHood/KA", ShooterHoodConstants.kA);

  LoggedTunableNumber maxA =
      new LoggedTunableNumber("ShooterHood/acceleration", ShooterHoodConstants.maxAcceleration);
  LoggedTunableNumber maxV =
      new LoggedTunableNumber("ShooterHood/velocity", ShooterHoodConstants.maxVelocity);
  LoggedTunableNumber maxJerk =
      new LoggedTunableNumber("ShooterHood/jerk", ShooterHoodConstants.maxJerk);

  public static LoggedTunableNumber hoodAngle =
      new LoggedTunableNumber("Auto Shoot/Hood Angle", -0.04);

  private double goal = 0.0;

  public ShooterHoodIOReal() {
    motorConfig.Audio.BeepOnBoot = true;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = ShooterHoodConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterHoodConstants.CURRENT_LIMIT;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.Feedback.SensorToMechanismRatio = ShooterHoodConstants.GEARING;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterHoodConstants.maxAngle;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterHoodConstants.minAngle;

    motorConfig.Slot0.kA = ShooterHoodConstants.kA;
    motorConfig.Slot0.kV = ShooterHoodConstants.kV;
    motorConfig.Slot0.kS = ShooterHoodConstants.kS;
    motorConfig.Slot0.kP = ShooterHoodConstants.kP;
    motorConfig.Slot0.kI = ShooterHoodConstants.kI;
    motorConfig.Slot0.kD = ShooterHoodConstants.kD;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = ShooterHoodConstants.maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = ShooterHoodConstants.maxAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = ShooterHoodConstants.maxJerk;

    shooterHoodMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public double getPosition() {
    return shooterHoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getGoal() {
    return this.goal;
  }

  @Override
  public void setVoltage(double volts) {
    shooterHoodMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setGoal(double goal) {
    this.goal = MathUtil.clamp(goal, ShooterHoodConstants.minAngle, ShooterHoodConstants.maxAngle);
    shooterHoodMotor.setControl(m_request.withPosition(this.goal));
  }

  @Override
  public void zero() {
    shooterHoodMotor.setPosition(0);
  }

  @Override
  public boolean readyToShoot() {
    return Math.abs(getPosition() - getGoal()) < ShooterHoodConstants.marginOfError;
  }

  @Override
  public void configure() {
    motorConfig.Slot0.kA = KA.get();
    motorConfig.Slot0.kV = KV.get();
    motorConfig.Slot0.kS = KS.get();
    motorConfig.Slot0.kP = KP.get();
    motorConfig.Slot0.kI = KI.get();
    motorConfig.Slot0.kD = KD.get();

    motorConfig.MotionMagic.MotionMagicAcceleration = maxA.get();
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = maxV.get();
    motorConfig.MotionMagic.MotionMagicJerk = maxJerk.get();

    shooterHoodMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocityRPS, current, voltage);

    inputs.goalRotations = this.goal;
    inputs.position = shooterHoodMotor.getPosition().getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
    inputs.current = current.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
  }
}
