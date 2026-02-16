package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.LoggedTunableNumber;

public class TurretIOReal implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.CAN_ID);

  private TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  private final StatusSignal<Angle> position = turretMotor.getPosition();
  private final StatusSignal<Current> current = turretMotor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = turretMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = turretMotor.getVelocity();

  LoggedTunableNumber KP = new LoggedTunableNumber("Turret/KP", TurretConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("Turret/KI", TurretConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("Turret/KD", TurretConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("Turret/KS", TurretConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("Turret/KV", TurretConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("Turret/KA", TurretConstants.kA);

  LoggedTunableNumber maxA =
      new LoggedTunableNumber("Turret/acceleration", TurretConstants.maxAcceleration);
  LoggedTunableNumber maxV =
      new LoggedTunableNumber("Turret/velocity", TurretConstants.maxVelocity);
  LoggedTunableNumber maxJerk = new LoggedTunableNumber("Turret/jerk", TurretConstants.maxJerk);

  private MotionMagicVoltage control = new MotionMagicVoltage(0);
  private MotionMagicConfigs controlConfig = new MotionMagicConfigs();

  public double goal;

  public TurretIOReal() {
    turretConfig.Slot0.kA = TurretConstants.kA;
    turretConfig.Slot0.kV = TurretConstants.kV;
    turretConfig.Slot0.kS = TurretConstants.kS;
    turretConfig.Slot0.kP = TurretConstants.kP;
    turretConfig.Slot0.kI = TurretConstants.kI;
    turretConfig.Slot0.kD = TurretConstants.kD;

    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = 60;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretConfig.CurrentLimits.SupplyCurrentLimit = 60;

    turretConfig.Feedback.SensorToMechanismRatio = TurretConstants.gearing;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.maxAngle / 360.0;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.minAngle / 360.0;

    turretMotor.getConfigurator().apply(turretConfig);

    controlConfig.MotionMagicExpo_kA = TurretConstants.maxAcceleration;
    controlConfig.MotionMagicExpo_kV = TurretConstants.maxVelocity;
    controlConfig.MotionMagicJerk = TurretConstants.maxJerk;

    turretMotor.getConfigurator().apply(controlConfig);
    zero();
  }

  @Override
  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getGoal() {
    return this.goal;
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setGoal(double goal) {
    this.goal =
        MathUtil.clamp(goal, TurretConstants.minAngle / 360.0, TurretConstants.maxAngle / 360.0);
    turretMotor.setControl(control.withPosition(this.goal));
  }

  @Override
  public void zero() {
    turretMotor.setPosition(0);
  }

  @Override
  public void configure() {
    turretConfig.Slot0.kA = KA.get();
    turretConfig.Slot0.kV = KV.get();
    turretConfig.Slot0.kS = KS.get();
    turretConfig.Slot0.kP = KP.get();
    turretConfig.Slot0.kI = KI.get();
    turretConfig.Slot0.kD = KD.get();

    controlConfig.MotionMagicAcceleration = maxA.get();
    controlConfig.MotionMagicCruiseVelocity = maxV.get();
    controlConfig.MotionMagicJerk = maxJerk.get();

    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.getConfigurator().apply(controlConfig);
  }

  @Override
  public boolean readyToShoot() {
    return Math.abs(getPosition() - getGoal()) < TurretConstants.marginOfError;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, current, voltage, velocity);

    inputs.goal = this.goal;
    inputs.position = position.getValueAsDouble();
    inputs.current = current.getValueAsDouble();
    inputs.volts = voltage.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
  }
}
