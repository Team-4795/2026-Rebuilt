package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOTalon implements IntakeIO {
  TalonFX motor1 = new TalonFX(62);
  TalonFX motor2 = new TalonFX(61);

  LoggedTunableNumber KP = new LoggedTunableNumber("Intake/KP", IntakeConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("Intake/KI", IntakeConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("Intake/KD", IntakeConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("Intake/KS", IntakeConstants.kS);
  LoggedTunableNumber KV = new LoggedTunableNumber("Intake/KV", IntakeConstants.kV);
  LoggedTunableNumber KA = new LoggedTunableNumber("Intake/KA", IntakeConstants.kA);

  // LoggedTunableNumber rps = new Logge

  TalonFXConfiguration config = new TalonFXConfiguration();

  VelocityTorqueCurrentFOC control = new VelocityTorqueCurrentFOC(0);

  private final StatusSignal<AngularVelocity> rps1 = motor1.getVelocity();
  private final StatusSignal<AngularVelocity> rps2 = motor2.getVelocity();
  private final StatusSignal<Current> current1 = motor1.getTorqueCurrent();
  private final StatusSignal<Current> current2 = motor2.getTorqueCurrent();

  private double goalRPS = 0.0;

  public IntakeIOTalon() {
    BaseStatusSignal.setUpdateFrequencyForAll(20, rps1, rps2, current1, current2);

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.SupplyCurrentLimit = 70;

    config.Feedback.SensorToMechanismRatio = IntakeConstants.GEARING;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configure();
  }

  @Override
  public void setIntakeVoltage(double volts) {
    // motor1.setVoltage(volts);
    // motor2.setVoltage(volts);

    motor1.setControl(new VoltageOut(volts).withEnableFOC(true));
    motor2.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void setGoalRPS(double rps) {
    rps = MathUtil.clamp(rps, 0, IntakeConstants.maxRPS);
    this.goalRPS = rps;

    motor1.setControl(control.withVelocity(rps));
    motor2.setControl(control.withVelocity(rps));
  }

  @Override
  public void configure() {
    config.Slot0.kS = KS.get();
    config.Slot0.kV = KV.get();
    config.Slot0.kA = KA.get();
    config.Slot0.kP = KP.get();
    config.Slot0.kI = KI.get();
    config.Slot0.kD = KD.get();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor1.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor2.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(rps1, rps2, current1, current2);

    inputs.angularVelocityRPSA = rps1.getValueAsDouble();
    inputs.angularVelocityRPSB = rps2.getValueAsDouble();
    inputs.currentAmpsA = current1.getValueAsDouble();
    inputs.currentAmpsB = current2.getValueAsDouble();
    inputs.goalRPS = goalRPS;
  }
}
