package frc.robot.subsystems.ShooterHood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterHoodIOReal implements ShooterHoodIO {
  private TalonFX shooterHoodMotor = new TalonFX(ShooterHoodConstants.CAN_ID);

  private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private final StatusSignal<Angle> position = shooterHoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRPS = shooterHoodMotor.getVelocity();
  private final StatusSignal<Current> current = shooterHoodMotor.getTorqueCurrent();
  private final StatusSignal<Voltage> voltage = shooterHoodMotor.getMotorVoltage();

  private double goal = 0.0;

  public ShooterHoodIOReal() {
    motorConfig.Audio.BeepOnBoot = true;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = ShooterHoodConstants.CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = ShooterHoodConstants.CURRENT_LIMIT;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

    motorConfig.MotionMagic.MotionMagicAcceleration = ShooterHoodConstants.maxVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = ShooterHoodConstants.maxAcceleration;
    motorConfig.MotionMagic.MotionMagicJerk = ShooterHoodConstants.maxJerk;

    shooterHoodMotor.getConfigurator().apply(motorConfig);
  }

  @Override
  public double getPosition() {
    return shooterHoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    shooterHoodMotor.setVoltage(volts);
  }

  @Override
  public void setGoal(double goal) {
    this.goal = goal;
    shooterHoodMotor.setControl(m_request.withPosition(goal));
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocityRPS, current, voltage);

    inputs.goalRotations = this.goal;
    inputs.position = position.getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
    inputs.current = current.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
  }
}
