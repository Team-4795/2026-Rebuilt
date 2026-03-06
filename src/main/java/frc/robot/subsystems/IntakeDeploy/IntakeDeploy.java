package frc.robot.subsystems.IntakeDeploy;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {
  private IntakeDeployIO io;
  private IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();
  private static IntakeDeploy instance;

  public static IntakeDeploy initialize(IntakeDeployIO io) {
    if (instance == null) {
      instance = new IntakeDeploy(io);
    }
    return instance;
  }

  public static IntakeDeploy getInstance() {
    return instance;
  }

  private IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    io.updateInputs(inputs);

    setDefaultCommand(
        Commands.run(
            () -> {
              io.updateMotionProfile();

              //   double up =
              //       MathUtil.applyDeadband(
              //           OIConstants.operatorController.getRightTriggerAxis(),
              //           OIConstants.kAxisDeadband);
              //   double down =
              //       MathUtil.applyDeadband(
              //           OIConstants.operatorController.getLeftTriggerAxis(),
              //           OIConstants.kAxisDeadband);

              //   double change = 0.01 * (Math.pow(up, 3) - Math.pow(down, 3));
              //   if (DriverStation.isTeleop()) {
              //     setGoal(inputs.deployMotorGoal + change);
              //   }
              // },
            },
            this));
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  // method to set voltage of deploy motor
  public void setDeployVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void zero() {
    io.zero();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);
  }
}
