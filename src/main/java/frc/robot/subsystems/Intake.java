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
  public void setIntakeVoltage(double volts) {
    io.setVoltage(volts);
  }

  // method to set speed of deploy motor
  public void setDeployVoltage(double volts) {
    io.setDeployVoltage(volts);
  }

  // Command to set speed of both motors
  public Command setIntakeVoltageCommand(double volts) {
    return runOnce(() -> setIntakeVoltage(volts));
  }

  // Spin motors max speed
  public Command intake() {
    return Commands.startEnd(() -> setIntakeVoltage(6), () -> setIntakeVoltage(0), this);
  }

  // Spin motors max speed opposite direction
  public Command reverseIntake() {
    return Commands.startEnd(() -> setIntakeVoltage(-6), () -> setIntakeVoltage(0), this);
  }

  // Stop motors from spinning
  public Command stop() {
    return setIntakeVoltageCommand(0);
  }

  // deploy intake by running the deploy motor
  public Command deployIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> setDeployVoltage(6)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> setDeployVoltage(0)));
  }

  // retract intake
  public Command retractIntake() {
    return Commands.sequence(
        Commands.runOnce(() -> setDeployVoltage(-6)),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> setDeployVoltage(0)));
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
