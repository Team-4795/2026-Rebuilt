package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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
  }

  // Method to set speed of both motors
  public void setIntakeSpeed(double speed) {
    io.setSpeed(speed);
  }

  // Command to set speed of both motors
  public Command setIntakeSpeedCommand(double speed) {
    return runOnce(() -> setIntakeSpeed(speed));
  }

  // Spin motors max speed
  public Command intake() {
    return Commands.startEnd(() -> setIntakeSpeed(1), () -> setIntakeSpeed(0), this);
  }

  // Spin motors max speed opposite direction
  public Command reverseIntake() {
    return Commands.startEnd(() -> setIntakeSpeed(-1), () -> setIntakeSpeed(0), this);
  }

  // Stop motors from spinning
  public Command stop() {
    return setIntakeSpeedCommand(0);
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
