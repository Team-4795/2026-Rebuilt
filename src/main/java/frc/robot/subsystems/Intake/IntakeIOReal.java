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

  private SparkFlexConfig intakeConfig = new SparkFlexConfig();

  private double intakeVolts = 0;

  public IntakeIOReal() {
    // Intake motor config
    intakeMotorA.clearFaults();
    intakeMotorB.clearFaults();

    intakeConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeConfig.idleMode(IdleMode.kCoast);

    intakeMotorA.setCANTimeout(20);
    intakeMotorB.setCANTimeout(20);

    intakeMotorA.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotorB.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setIntakeVoltage(double v) {
    intakeVolts = v;
    intakeMotorA.setVoltage(intakeVolts);
    intakeMotorB.setVoltage(intakeVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltageA = intakeVolts;
    inputs.angularVelocityRPSA = intakeMotorA.getEncoder().getVelocity();
    inputs.currentAmpsA = intakeMotorA.getOutputCurrent();

    inputs.voltageB = intakeVolts;
    inputs.angularVelocityRPSB = intakeMotorB.getEncoder().getVelocity();
    inputs.currentAmpsB = intakeMotorB.getOutputCurrent();
  }
}
