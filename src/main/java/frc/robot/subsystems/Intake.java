package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
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

    setDefaultCommand(
        Commands.run(
            () -> {
              double change =
                  MathUtil.applyDeadband(
                      -OIConstants.operatorController.getRightY(),
                      OIConstants.OperatorLAxisDeadband);
              change = .05 * Math.pow(change, 3);
              if (DriverStation.isTeleopEnabled() && change != 0) {
                io.setGoal(inputs.deployMotorGoal + change);
              }
              io.updateMotionProfile();
            },
            this));
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
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
