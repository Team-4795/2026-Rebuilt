package frc.robot.subsystems;

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

  public void setIntakeSpeed(double speed) {
    io.setSpeed(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // TODO finish this later
  }
}
