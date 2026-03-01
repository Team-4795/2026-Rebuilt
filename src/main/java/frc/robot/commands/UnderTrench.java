package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class UnderTrench extends Command {
  private Drive drive = Drive.getInstance();
  private CommandXboxController driverController;

  private ProfiledPIDController rotationController;
  private ProfiledPIDController translationController;

  private final TrapezoidProfile.Constraints rotationConstraints;
  private final TrapezoidProfile.Constraints translationConstraints;

  private Pose2d currentPose;
  private Translation2d targetTranslation;
  private double robotY;
  private double targetY;

  private double distance;
  private double mult = 1.0;

  private double targetRotation;

  public UnderTrench() {
    driverController = Constants.OIConstants.driverController;
    rotationConstraints = new TrapezoidProfile.Constraints(drive.getMaxAngularSpeedRadPerSec(), 40);
    rotationController = new ProfiledPIDController(3, 0, 0.1, rotationConstraints);

    translationConstraints =
        new TrapezoidProfile.Constraints(drive.getMaxLinearSpeedMetersPerSec(), 10);
    translationController = new ProfiledPIDController(2, 0, 0, translationConstraints);
  }

  @Override
  public void initialize() {
    DriverStation.getAlliance()
        .ifPresent(
            (alliance) -> {
              mult = (alliance == Alliance.Red) ? 1.0 : -1.0;
            });

    currentPose = drive.getPose();
    targetTranslation = currentPose.getTranslation().nearest(FieldConstants.trenchList);

    robotY = drive.getPose().getY();
    targetY = currentPose.getTranslation().nearest(FieldConstants.trenchList).getY();

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.reset(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        drive.getChassisSpeeds().omegaRadiansPerSecond);

    distance = Math.abs(targetY - robotY);
    translationController.reset(distance, 0);
  }

  @Override
  public void execute() {
    Translation2d nearestTrench = currentPose.getTranslation().nearest(FieldConstants.trenchList);

    if (nearestTrench.equals(Constants.FieldConstants.redLeftTrench)
        || nearestTrench.equals(Constants.FieldConstants.blueRightTrench)) {
      targetRotation = -Math.PI / 2.0;
      targetTranslation = nearestTrench.plus(new Translation2d(0, 0.06));
    } else {
      targetRotation = Math.PI / 2.0;
      targetTranslation = nearestTrench.plus(new Translation2d(0, -0.06));
    }

    robotY = drive.getPose().getY();
    targetY = targetTranslation.getY();

    currentPose = drive.getPose();
    distance = Math.abs(targetY - robotY);

    translationController.reset(distance, translationController.getSetpoint().velocity);

    // double scalar = scalar(distance);
    double drivePIDOutput = translationController.calculate(distance, 0);
    double driveVel = mult * (translationController.getSetpoint().velocity + drivePIDOutput);

    double rotationPIDOutput =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetRotation);
    double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            -driverController.getLeftY() * drive.getMaxLinearSpeedMetersPerSec(),
            driveVel * (targetTranslation.getY() - currentPose.getY()),
            omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  // private double scalar(double distance) {
  //   if (distance > 0.2) {
  //     return 1.0;
  //   } else if (0 < distance && distance < 0.2) {
  //     return MathUtil.clamp((1.0 / (0.2 - 0)) * (distance - 0), 0, 1);
  //   } else {
  //     return 0.0;
  //   }
  // }

  @Override
  public void end(boolean interrupted) {}
}
