package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static Intake instance;

  public static Intake initialize(IntakeIO io) {
    if (instance == null) {
      instance = new Intake(io);
    }
    return instance;
  }

  public static Intake getInstance() {
    return instance;
  }

  private Intake(IntakeIO io) {
    this.io = io;
    io.updateInputs(inputs);
    setDefaultCommand(Commands.run(() -> setIntakeVoltage(0), this)); // auto deploy
  }

  // Method to set speed of intake motors
  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
