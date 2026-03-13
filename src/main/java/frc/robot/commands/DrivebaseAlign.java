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
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DrivebaseAlign extends Command {
  private Drive drive = Drive.getInstance();

  private ProfiledPIDController rotationController;

  private final TrapezoidProfile.Constraints rotationConstraints;

  private Pose2d currentPose;
  private Pose2d targetPose;
  private Translation2d turretPose;

  private double targetRotation;

  public DrivebaseAlign() {
    rotationConstraints = new TrapezoidProfile.Constraints(drive.getMaxAngularSpeedRadPerSec(), 40);
    rotationController =
        new ProfiledPIDController(
            DriveConstants.turnKp, 0, DriveConstants.turnKd, rotationConstraints);
  }

  @Override
  public void initialize() {
    currentPose = drive.getPose();

    targetPose = StateManager.getInstance().getTargetPose();
    turretPose =
        currentPose
            .getTranslation()
            .plus(TurretConstants.OFFSET.rotateBy(currentPose.getRotation()));

    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.reset(
        MathUtil.angleModulus(currentPose.getRotation().getRadians()),
        drive.getChassisSpeeds().omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    currentPose = drive.getPose();

    targetPose = StateManager.getInstance().getTargetPose();
    turretPose =
        currentPose
            .getTranslation()
            .plus(TurretConstants.OFFSET.rotateBy(currentPose.getRotation()));

    double deltaY = targetPose.getY() - turretPose.getY();
    double deltaX = targetPose.getX() - turretPose.getX();

    targetRotation = (Math.atan2(deltaY, deltaX));
    double rotationPIDOutput =
        rotationController.calculate(
            MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetRotation);
    double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    Logger.recordOutput("Target", targetPose);
    Logger.recordOutput("Goal Angle", targetRotation);
  }

  @Override
  public void end(boolean interrupted) {}
}
