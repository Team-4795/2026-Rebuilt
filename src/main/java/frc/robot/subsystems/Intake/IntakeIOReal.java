package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

public class IntakeIOReal implements IntakeIO {
  // one motors on each side of the spinny bar thing
  private final PWMTalonFX intakeMotorA = new PWMTalonFX(IntakeConstants.PWMPort);
  private final PWMTalonFX intakeMotor2 = new PWMTalonFX(IntakeConstants.PWMPort2);

  private double intakeVolts = 0;

  public IntakeIOReal() {
    // Intake motor config
    intakeMotor2.setInverted(true);
    intakeMotorA.setInverted(false);
  }

  @Override
  public void setIntakeVoltage(double v) {
    intakeVolts = v;
    intakeMotorA.setVoltage(intakeVolts);
    intakeMotor2.setVoltage(intakeVolts);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltageA = intakeMotorA.getVoltage();
  }
}
