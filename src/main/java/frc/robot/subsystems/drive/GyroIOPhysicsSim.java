package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SwerveDrivePhysicsSim;

/** Gyro IO adapter backed by a shared centralized swerve simulation. */
public class GyroIOPhysicsSim implements GyroIO {
  private final SwerveDrivePhysicsSim sim;

  public GyroIOPhysicsSim(SwerveDrivePhysicsSim sim) {
    this.sim = sim;
  }

  @Override
  public void zeroHeading() {
    sim.zeroHeading();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    sim.updateToNow();
    SwerveDrivePhysicsSim.GyroSample sample = sim.getGyroSample();

    inputs.connected = true;
    inputs.yawPosition = sample.yaw();
    inputs.yawVelocityRadPerSec = sample.yawVelocityRadPerSec();
    inputs.odometryYawTimestamps = new double[] {sample.timestampSec()};
    inputs.odometryYawPositions = new Rotation2d[] {sample.yaw()};
  }

  public Pose2d getTruePose() {
    return sim.getTruePose();
  }
}
