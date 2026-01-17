package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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
  }

  // doesn't return command
  public void setIntakeSpeed(double speed) {
    io.setSpeed(speed);
  }

  // returns a command to make things simpler
  public Command setIntakeSpeedCommand(double speed) {
    return runOnce(() -> setIntakeSpeed(speed));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/Speed", io.getSpeed());
    Logger.recordOutput("Intake/Voltage", io.getVoltage());
    Logger.recordOutput("Intake/Current", io.getCurrent());
  }
}
