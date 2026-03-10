package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
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
    io.setVelocityRPS(velocityRPS);
  }

  public void setRPSDynamic() {
    double distance = Drive.getInstance().getTurretDistanceToTarget();
    double goalRPS = ShooterConstants.shooterVelocityHubMap.get(distance);

    // Logger.recordOutput("distance", distance);
    // Logger.recordOutput("goalRPS", goalRPS);
    setVelocityRPS(goalRPS);
  }

  public boolean readyToShoot() {
    return io.readyToShoot();
  }

  public void configure() {
    io.configure();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
