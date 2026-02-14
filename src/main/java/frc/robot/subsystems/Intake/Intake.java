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
    setDefaultCommand(Commands.run(() -> io.setGoal(IntakeConstants.stowPosition), this));
  }

  // Method to set speed of both motors
  public void setIntakeVoltage(double volts) {
    io.setIntakeVoltage(volts);
  }

  // method to set speed of deploy motor
  public void setDeployVoltage(double volts) {
    io.setDeployVoltage(volts);
  }

  public void setDeployGoal(double goal) {
    io.setGoal(goal);
  }

  @Override
  public void periodic() {
    io.updateMotionProfile();

    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
