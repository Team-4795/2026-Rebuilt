package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static Intake instance;

  LoggedTunableNumber RPS = new LoggedTunableNumber("Intake/RPS", 50);

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
    setDefaultCommand(Commands.run(() -> setIntakeVoltage(0), this));
  }

  // Method to set speed of intake motors
  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  public void setGoalRPS(double rps) {
    io.setGoalRPS(rps);
  }

  public void setFixedGoalRPS(){
    io.setGoalRPS(RPS.getAsDouble());
  }  

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
