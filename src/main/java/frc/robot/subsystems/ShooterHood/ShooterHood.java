package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.StateManager.StateManager.OperationStates;
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
  }

  public void setGoal(double goal) {
    if (!StateManager.OperationStates.inDecapitationZone) {
      io.setGoal(goal);
    }
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

  public void setGoalDynamic() {
    // Set goal if outside the box
    if (!OperationStates.inDecapitationZone) {
      io.setGoal(ShooterHoodConstants.shooterHoodMap.get(Drive.getInstance().getDistanceToHub()));
    }
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
