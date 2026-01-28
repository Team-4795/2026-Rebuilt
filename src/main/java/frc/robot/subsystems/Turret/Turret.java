package frc.robot.subsystems.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private static Turret instance;

  public static Turret getInstance() {
    return instance;
  }

  public static Turret initialize(TurretIO io) {
    if (instance == null) {
      instance = new Turret(io);
    }
    return instance;
  }

  private Turret(TurretIO turretIO) {
    this.io = turretIO;
    io.updateInputs(inputs);
  }

  public void setGoal(double goal) {
    io.setGoal(goal);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public double getTurretAngle() {
    return io.getPosition();
  }

  // Visualize the direction the turret aims in ascope
  public Pose2d visualizeTurret() {
    Translation2d turretOffsetPose = TurretConstants.OFFSET;
    Pose2d robotPose = Drive.getInstance().getPose();

    Translation2d turretTranslation =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    Pose2d turretPose =
        new Pose2d(
            turretTranslation.getX(),
            turretTranslation.getY(),
            new Rotation2d(
                -(Units.rotationsToRadians(getTurretAngle() + TurretConstants.angleOffset)
                    - robotPose.getRotation().getRadians())));

    return turretPose;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/Turret Visualization", visualizeTurret());
  }
}
