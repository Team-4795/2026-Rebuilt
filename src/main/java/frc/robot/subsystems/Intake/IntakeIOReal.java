package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakeIOReal implements IntakeIO {
  // one motors on each side of the spinny bar thing
  private final SparkFlex intakeMotorA =
      new SparkFlex(IntakeConstants.canIDIntakeA, MotorType.kBrushless);
  private final SparkFlex intakeMotorB =
      new SparkFlex(IntakeConstants.canIDIntakeB, MotorType.kBrushless);

  private SparkFlexConfig intakeConfigA = new SparkFlexConfig();
  private SparkFlexConfig intakeConfigB = new SparkFlexConfig();

  private double intakeVolts = 0;

  public IntakeIOReal() {
    // Intake motor config
    intakeMotorA.clearFaults();
    intakeMotorB.clearFaults();

    intakeConfigA.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeConfigA.idleMode(IdleMode.kCoast);

    intakeConfigA.voltageCompensation(12.0);
    intakeConfigA.inverted(false);

    intakeConfigB.apply(intakeConfigA);
    intakeConfigB.follow(intakeMotorA.getDeviceId(), true);

    intakeMotorA.configure(
        intakeConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorB.configure(
        intakeConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotorA.setCANTimeout(20);
    intakeMotorB.setCANTimeout(20);
  }

  @Override
  public void setIntakeVoltage(double v) {
    intakeVolts = v;
    intakeMotorA.setVoltage(intakeVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltageA = intakeVolts;
    inputs.angularVelocityRPSA = intakeMotorA.getEncoder().getVelocity() / 60.0;
    inputs.currentAmpsA = intakeMotorA.getOutputCurrent();

    inputs.voltageB = intakeVolts;
    inputs.angularVelocityRPSB = intakeMotorB.getEncoder().getVelocity() / 60.0;
    inputs.currentAmpsB = intakeMotorB.getOutputCurrent();
  }
}
