package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove extends Command {
  private final Turret turret;
  private final Drive drive;
  Pose2d robotPose;
  Translation2d hub = Constants.FieldConstants.redHub;
  Translation2d turretOffsetPose = TurretConstants.OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double velocityXOffset = 0;
  double velocityYOffset = 0;
  double velocityOmega = 0; // needs to be in rotations
  double omegaXOffset = 0;
  double omegaYOffset = 0;
  double desiredRot = 0;
  double turretAngle = 0;
  Translation2d velocityVector;
  double rotationOffset = TurretConstants.angleOffset; // zeroing offset
  double tAir = 1; // calculate this with utility class or interpolating tree
  double tLat; // time for indexer to actually shoot out a ball (latency)
  ChassisSpeeds fieldRelative;


  public ShootOnTheMove(Drive drive, Turret turret) {
    this.drive = drive;
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    robotPose = drive.getPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());
    
    double omega = drive.getChassisSpeeds().omegaRadiansPerSecond;

    velocityXOffset = fieldRelative.vxMetersPerSecond * tAir;
    velocityYOffset = fieldRelative.vyMetersPerSecond * tAir;

    //trig way 
    // omegaXOffset = -Math.sin(robotPose.getRotation().getRadians() + TurretConstants.robotRelativeAngleOffset) * TurretConstants.turretRadiusOffset * drive.getChassisSpeeds().omegaRadiansPerSecond * tAir;
    // omegaYOffset = Math.cos(robotPose.getRotation().getRadians() + TurretConstants.robotRelativeAngleOffset) * TurretConstants.turretRadiusOffset * drive.getChassisSpeeds().omegaRadiansPerSecond * tAir;

    //cross product here pretty sure this is the same thing as trig above. 
    omegaXOffset = -omega * turretOffsetPose.rotateBy(robotPose.getRotation()).getY();
    omegaYOffset = omega * turretOffsetPose.rotateBy(robotPose.getRotation()).getX(); 

    deltaX = hub.getX() - turretPose.getX() - velocityXOffset;
    deltaY = hub.getY() - turretPose.getY() - velocityYOffset;

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle -= TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);
    turret.setGoal(desiredRot);

    Logger.recordOutput("Aim At Hub/Hub Pose", hub);
    Logger.recordOutput("Aim At Hub/Desired Hub", new Pose2d(deltaX, deltaY, new Rotation2d()));
    Logger.recordOutput("Aim At Hub/Desired Rotation", desiredRot);
    Logger.recordOutput("Aim At Hub/ XOffset", deltaX);
    Logger.recordOutput("Aim At Hub/ YOffset", deltaY);
    Logger.recordOutput("Aim At Hub/ OmegaXOffset", omegaXOffset);
    Logger.recordOutput("Aim At Hub/ OmegaYOffset", omegaYOffset);
    Logger.recordOutput("Aim At Hub/ LinearXOffset", velocityXOffset);
    Logger.recordOutput("Aim At Hub/ LinearYOffset", velocityYOffset);


  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
