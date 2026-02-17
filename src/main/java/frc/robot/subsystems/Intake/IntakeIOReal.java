package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;

public class IntakeIOReal implements IntakeIO {
  // one motors on each side of the spinny bar thing
  private final PWMSparkFlex intakeMotorA = new PWMSparkFlex(IntakeConstants.PWMPort);

  private double intakeVolts = 0;

  public IntakeIOReal() {
    // Intake motor config
    intakeMotorA.setSafetyEnabled(true);
  }

  @Override
  public void setIntakeVoltage(double v) {
    intakeVolts = v;
    intakeMotorA.setVoltage(intakeVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltageA = intakeMotorA.getVoltage();
  }
}
