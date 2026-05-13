package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.StateManager.StateManager;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static Shooter instance;

  public static Shooter getInstance() {
    return instance;
  }

  public static Shooter initialize(ShooterIO io) {
    if (instance == null) {
      instance = new Shooter(io);
    }
    return instance;
  }

  private Shooter(ShooterIO shooterIO) {
    io = shooterIO;
    io.updateInputs(inputs);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocityRPS(double velocityRPS) {
    if (StateManager.getInstance().canShooterRev()) {
      io.setVelocityRPS(velocityRPS);
    } else {
      io.setVelocityRPS(0);
    }
  }

  public boolean readyToShoot() {
    return io.shootNow();
  }

  public void configure() {
    io.configure();
  }

  public void resetShooter() {
    io.resetShooter();
  }

  public void setOutreachRPS(double rps) {
    io.outreachRPS(rps);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
