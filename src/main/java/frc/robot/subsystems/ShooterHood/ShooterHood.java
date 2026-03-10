package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {
  private ShooterHoodIO io;
  private ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  private static ShooterHood instance;

  public static ShooterHood getInstance() {
    return instance;
  }

  public static ShooterHood initialize(ShooterHoodIO io) {
    if (instance == null) {
      instance = new ShooterHood(io);
    }
    return instance;
  }

  private ShooterHood(ShooterHoodIO shooterHoodIO) {
    io = shooterHoodIO;
    io.updateInputs(inputs);

    setDefaultCommand(Commands.run(() -> setGoal(0), this));

    // setDefaultCommand(Commands.run(() -> setGoal(inputs.goalRotations), this));
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  public void setGoalDynamic() {
    io.setGoal(
        ShooterHoodConstants.shooterHoodHubMap.get(Drive.getInstance().getTurretDistanceToTarget()));
  }

  public void zero() {
    io.zero();
  }

  public boolean readyToShoot() {
    return io.readyToShoot();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void configure() {
    io.configure();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Hood", inputs);
  }
}
