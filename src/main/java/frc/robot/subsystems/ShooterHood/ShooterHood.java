package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.util;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends SubsystemBase {
  private ShooterHoodIO io;
  private ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();

  private static ShooterHood instance;
  Translation2d tL = new Translation2d();
  Translation2d tR = new Translation2d();
  Translation2d bL = new Translation2d();
  Translation2d bR = new Translation2d();

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
    io.setGoal(goal);
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
    Pose2d robotPose = Drive.getInstance().getPose();
    Translation2d robotTranslation = robotPose.getTranslation();

    Translation2d closest = robotTranslation.nearest(FieldConstants.trenchList);

    // box dimensions scale with velocity
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            Drive.getInstance().getChassisSpeeds(), robotPose.getRotation());

    // dimensions of no auto score zone
    double boxXDim =
        1 + ShooterHoodConstants.boxXMultiplier * Math.abs(fieldRelative.vxMetersPerSecond);
    double boxYDim =
        FieldConstants.trenchWidth
            + ShooterHoodConstants.boxYMultiplier * Math.abs(fieldRelative.vyMetersPerSecond);

    Translation2d topLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() + boxYDim);
    Translation2d topRight = new Translation2d(closest.getX() + boxXDim, closest.getY() + boxYDim);
    Translation2d botLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() - boxYDim);
    Translation2d botRight = new Translation2d(closest.getX() + boxXDim, closest.getY() - boxYDim);

    tL = topLeft;
    tR = topRight;
    bL = botLeft;
    bR = botRight;

    // set goal if outside the box
    if (!util.inBetween(robotTranslation.getX(), topLeft.getX(), topRight.getX())
        && (!util.inBetween(robotTranslation.getY(), topLeft.getY(), botLeft.getY()))) {
      io.setGoal(ShooterHoodConstants.shooterHoodMap.get(Drive.getInstance().getDistanceToHub()));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Hood", inputs);

    Logger.recordOutput("Box Top Right", new Pose2d(tR, new Rotation2d()));
    Logger.recordOutput("Box Bottom Left", new Pose2d(bL, new Rotation2d()));
    Logger.recordOutput("Box Top Left", new Pose2d(tL, new Rotation2d()));
    Logger.recordOutput("Box Bottom Right", new Pose2d(bR, new Rotation2d()));
  }
}
