package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
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

    // the 1 can be changed if we want a minimum box width
    double boxXDim = 1 + ShooterHoodConstants.boxXMultiplier * fieldRelative.vxMetersPerSecond;
    double boxYDim =
        FieldConstants.trenchWidth
            + ShooterHoodConstants.boxYMultiplier * fieldRelative.vyMetersPerSecond;

    double topRightX = closest.getX() + boxXDim;
    double topRightY = closest.getY() + boxYDim;
    double bottomLeftX = closest.getX() - boxXDim;
    double bottomLeftY = closest.getY() - boxYDim;

    Translation2d topRight = new Translation2d(topRightX, topRightY);
    Translation2d bottomLeft = new Translation2d(bottomLeftX, bottomLeftY);

    Logger.recordOutput("Box Top Right", topRight);
    Logger.recordOutput("Box Bottom Left", bottomLeft);

    // also probably need scaling based on direction

    if ((robotTranslation.getX() > bottomLeftX)
        && (robotTranslation.getY() > bottomLeftY)
        && (robotTranslation.getX() < topRightX)
        && (robotTranslation.getY() < topRightY)) {
      io.setGoal(ShooterHoodConstants.shooterHoodMap.get(Drive.getInstance().getDistanceToHub()));
    }

    // else?
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Hood", inputs);
  }
}
