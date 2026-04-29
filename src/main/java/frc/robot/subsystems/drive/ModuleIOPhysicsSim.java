package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SwerveDrivePhysicsSim;

/** Module IO adapter backed by a shared centralized swerve simulation. */
public class ModuleIOPhysicsSim implements ModuleIO {
  private final SwerveDrivePhysicsSim sim;
  private final int moduleIndex;

  public ModuleIOPhysicsSim(SwerveDrivePhysicsSim sim, int moduleIndex) {
    this.sim = sim;
    this.moduleIndex = moduleIndex;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    sim.updateToNow();
    SwerveDrivePhysicsSim.ModuleSample sample = sim.getModuleSample(moduleIndex);

    inputs.driveConnected = true;
    inputs.drivePositionRad = sample.drivePositionRad();
    inputs.driveVelocityRadPerSec = sample.driveVelocityRadPerSec();
    inputs.driveAppliedVolts = sample.driveAppliedVolts();
    inputs.driveCurrentAmps = sample.driveCurrentAmps();

    inputs.turnConnected = true;
    inputs.turnPosition = sample.turnPosition();
    inputs.turnVelocityRadPerSec = sample.turnVelocityRadPerSec();
    inputs.turnAppliedVolts = sample.turnAppliedVolts();
    inputs.turnCurrentAmps = sample.turnCurrentAmps();

    inputs.odometryTimestamps = new double[] {sample.timestampSec()};
    inputs.odometryDrivePositionsRad = new double[] {sample.drivePositionRad()};
    inputs.odometryTurnPositions = new Rotation2d[] {sample.turnPosition()};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    sim.setDriveOpenLoop(moduleIndex, output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    sim.setTurnOpenLoop(moduleIndex, output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    sim.setDriveVelocitySetpoint(moduleIndex, velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    sim.setTurnPositionSetpoint(moduleIndex, rotation);
  }
}
