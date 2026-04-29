package frc.robot.util;

import static frc.robot.subsystems.drive.DriveConstants.driveMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.driveMotorReduction;
import static frc.robot.subsystems.drive.DriveConstants.driveSimKv;
import static frc.robot.subsystems.drive.DriveConstants.driveSimP;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;
import static frc.robot.subsystems.drive.DriveConstants.robotMOI;
import static frc.robot.subsystems.drive.DriveConstants.robotMassKg;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorReduction;
import static frc.robot.subsystems.drive.DriveConstants.turnSimP;
import static frc.robot.subsystems.drive.DriveConstants.wheelCOF;
import static frc.robot.subsystems.drive.DriveConstants.wheelRadiusMeters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;

/** Centralized coupled swerve physics simulation. */
public class SwerveDrivePhysicsSim {
  public static final int MODULE_COUNT = 4;

  private static final double NOMINAL_VOLTAGE = 12.0;
  private static final double GRAVITY = 9.80665;
  private static final double INTERNAL_STEP_SEC = 1.0 / 200.0;
  private static final double MIN_UPDATE_STEP_SEC = 1e-4;
  private static final double DRIVE_WHEEL_MOI = 0.025;
  private static final double TURN_MOI = 0.004;
  private static final double KINETIC_FRICTION_RATIO = 0.85;
  private static final double STATIC_BLEND_VEL = 0.04;
  private static final double LONGITUDINAL_STIFFNESS_N_PER_MPS = 350.0;
  private static final double LATERAL_STIFFNESS_N_PER_MPS = 520.0;

  private final DCMotor driveMotor =
      frc.robot.subsystems.drive.DriveConstants.driveGearbox.withReduction(driveMotorReduction);
  private final DCMotor turnMotor =
      frc.robot.subsystems.drive.DriveConstants.turnGearbox.withReduction(turnMotorReduction);

  private final ModuleControl[] controls = new ModuleControl[MODULE_COUNT];

  private final SimState state = new SimState();

  private double headingOffsetRad = 0.0;
  private double lastTimestampSec = Timer.getFPGATimestamp();
  private double sampleTimestampSec = lastTimestampSec;

  public SwerveDrivePhysicsSim() {
    for (int i = 0; i < MODULE_COUNT; i++) {
      controls[i] = new ModuleControl();
    }
  }

  public synchronized void updateToNow() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestampSec;
    if (dt <= MIN_UPDATE_STEP_SEC) {
      return;
    }

    while (dt > 1e-9) {
      double step = Math.min(INTERNAL_STEP_SEC, dt);
      integrateRk4(step);
      dt -= step;
    }

    lastTimestampSec = now;
    sampleTimestampSec = now;
  }

  public synchronized void zeroHeading() {
    headingOffsetRad = state.headingRad;
  }

  public synchronized void setDriveOpenLoop(int module, double volts) {
    ModuleControl control = controls[module];
    control.driveClosedLoop = false;
    control.driveOpenLoopVolts = volts;
  }

  public synchronized void setTurnOpenLoop(int module, double volts) {
    ModuleControl control = controls[module];
    control.turnClosedLoop = false;
    control.turnOpenLoopVolts = volts;
  }

  public synchronized void setDriveVelocitySetpoint(int module, double velocityRadPerSec) {
    ModuleControl control = controls[module];
    control.driveClosedLoop = true;
    control.driveVelocitySetpointRadPerSec = velocityRadPerSec;
    control.driveFeedforwardVolts = driveSimKv * velocityRadPerSec;
  }

  public synchronized void setTurnPositionSetpoint(int module, Rotation2d setpoint) {
    ModuleControl control = controls[module];
    control.turnClosedLoop = true;
    control.turnPositionSetpointRad = setpoint.getRadians();
  }

  public synchronized ModuleSample getModuleSample(int module) {
    ModuleState m = state.modules[module];
    return new ModuleSample(
        m.drivePositionRad,
        m.driveVelocityRadPerSec,
        m.driveAppliedVolts,
        Math.abs(m.driveCurrentAmps),
        new Rotation2d(m.turnPositionRad),
        m.turnVelocityRadPerSec,
        m.turnAppliedVolts,
        Math.abs(m.turnCurrentAmps),
        sampleTimestampSec);
  }

  public synchronized GyroSample getGyroSample() {
    return new GyroSample(
        new Rotation2d(state.headingRad - headingOffsetRad),
        state.omegaRadPerSec,
        sampleTimestampSec);
  }

  public synchronized Pose2d getTruePose() {
    return new Pose2d(
        state.xMeters, state.yMeters, new Rotation2d(state.headingRad - headingOffsetRad));
  }

  public synchronized void setTruePose(Pose2d pose) {
    state.xMeters = pose.getX();
    state.yMeters = pose.getY();
    state.headingRad = pose.getRotation().getRadians() + headingOffsetRad;
    state.vxRobotMetersPerSec = 0.0;
    state.vyRobotMetersPerSec = 0.0;
    state.omegaRadPerSec = 0.0;
    lastTimestampSec = Timer.getFPGATimestamp();
    sampleTimestampSec = lastTimestampSec;
  }

  private void integrateRk4(double dt) {
    SimDerivative k1 = derivative(state);
    SimState s2 = state.plus(k1, dt * 0.5);
    SimDerivative k2 = derivative(s2);
    SimState s3 = state.plus(k2, dt * 0.5);
    SimDerivative k3 = derivative(s3);
    SimState s4 = state.plus(k3, dt);
    SimDerivative k4 = derivative(s4);
    state.integrate(k1, k2, k3, k4, dt);
    updateElectricalTelemetry();
  }

  private SimDerivative derivative(SimState s) {
    SimDerivative deriv = new SimDerivative();

    deriv.xMetersPerSec =
        s.vxRobotMetersPerSec * Math.cos(s.headingRad)
            - s.vyRobotMetersPerSec * Math.sin(s.headingRad);
    deriv.yMetersPerSec =
        s.vxRobotMetersPerSec * Math.sin(s.headingRad)
            + s.vyRobotMetersPerSec * Math.cos(s.headingRad);
    deriv.headingRadPerSec = s.omegaRadPerSec;

    double normalForcePerWheel = (robotMassKg * GRAVITY) / MODULE_COUNT;
    double maxStaticForce = wheelCOF * normalForcePerWheel;
    double maxKineticForce = maxStaticForce * KINETIC_FRICTION_RATIO;

    double totalFx = 0.0;
    double totalFy = 0.0;
    double totalTau = 0.0;

    for (int i = 0; i < MODULE_COUNT; i++) {
      ModuleState module = s.modules[i];
      ModuleControl control = controls[i];

      double driveVoltsCmd = computeDriveVoltage(control, module.driveVelocityRadPerSec);
      double turnVoltsCmd = computeTurnVoltage(control, module.turnPositionRad);

      double driveCurrent = driveMotor.getCurrent(module.driveVelocityRadPerSec, driveVoltsCmd);
      driveCurrent = MathUtil.clamp(driveCurrent, -driveMotorCurrentLimit, driveMotorCurrentLimit);
      double driveTorque = driveMotor.getTorque(driveCurrent);
      double commandedLongitudinalForce = driveTorque / wheelRadiusMeters;

      double turnCurrent = turnMotor.getCurrent(module.turnVelocityRadPerSec, turnVoltsCmd);
      turnCurrent = MathUtil.clamp(turnCurrent, -turnMotorCurrentLimit, turnMotorCurrentLimit);
      double turnTorque = turnMotor.getTorque(turnCurrent);

      double wheelHeading = module.turnPositionRad;
      double c = Math.cos(wheelHeading);
      double sn = Math.sin(wheelHeading);
      double ux = c;
      double uy = sn;
      double nx = -sn;
      double ny = c;

      double rx = moduleTranslations[i].getX();
      double ry = moduleTranslations[i].getY();
      double wheelVx = s.vxRobotMetersPerSec - s.omegaRadPerSec * ry;
      double wheelVy = s.vyRobotMetersPerSec + s.omegaRadPerSec * rx;

      double wheelSurfaceSpeed = module.driveVelocityRadPerSec * wheelRadiusMeters;
      double longitudinalSlip = wheelSurfaceSpeed - (wheelVx * ux + wheelVy * uy);
      double lateralSlip = wheelVx * nx + wheelVy * ny;

      double longitudinalForce =
          computeFrictionForce(
              commandedLongitudinalForce + LONGITUDINAL_STIFFNESS_N_PER_MPS * longitudinalSlip,
              longitudinalSlip,
              maxStaticForce,
              maxKineticForce);
      double lateralForce =
          computeFrictionForce(
              -LATERAL_STIFFNESS_N_PER_MPS * lateralSlip,
              lateralSlip,
              maxStaticForce,
              maxKineticForce);

      double fx = ux * longitudinalForce + nx * lateralForce;
      double fy = uy * longitudinalForce + ny * lateralForce;
      totalFx += fx;
      totalFy += fy;
      totalTau += rx * fy - ry * fx;

      deriv.modules[i].drivePositionRadPerSec = module.driveVelocityRadPerSec;
      deriv.modules[i].driveVelocityRadPerSecSq =
          (driveTorque - longitudinalForce * wheelRadiusMeters) / DRIVE_WHEEL_MOI;
      deriv.modules[i].turnPositionRadPerSec = module.turnVelocityRadPerSec;
      deriv.modules[i].turnVelocityRadPerSecSq = turnTorque / TURN_MOI;
    }

    deriv.vxRobotMetersPerSecSq = totalFx / robotMassKg + s.omegaRadPerSec * s.vyRobotMetersPerSec;
    deriv.vyRobotMetersPerSecSq = totalFy / robotMassKg - s.omegaRadPerSec * s.vxRobotMetersPerSec;
    deriv.omegaRadPerSecSq = totalTau / robotMOI;

    return deriv;
  }

  private void updateElectricalTelemetry() {
    for (int i = 0; i < MODULE_COUNT; i++) {
      ModuleState module = state.modules[i];
      ModuleControl control = controls[i];

      double driveVoltsCmd = computeDriveVoltage(control, module.driveVelocityRadPerSec);
      double driveCurrent = driveMotor.getCurrent(module.driveVelocityRadPerSec, driveVoltsCmd);
      driveCurrent = MathUtil.clamp(driveCurrent, -driveMotorCurrentLimit, driveMotorCurrentLimit);
      double driveTorque = driveMotor.getTorque(driveCurrent);
      module.driveAppliedVolts =
          MathUtil.clamp(
              driveMotor.getVoltage(driveTorque, module.driveVelocityRadPerSec),
              -NOMINAL_VOLTAGE,
              NOMINAL_VOLTAGE);
      module.driveCurrentAmps = driveCurrent;

      double turnVoltsCmd = computeTurnVoltage(control, module.turnPositionRad);
      double turnCurrent = turnMotor.getCurrent(module.turnVelocityRadPerSec, turnVoltsCmd);
      turnCurrent = MathUtil.clamp(turnCurrent, -turnMotorCurrentLimit, turnMotorCurrentLimit);
      double turnTorque = turnMotor.getTorque(turnCurrent);
      module.turnAppliedVolts =
          MathUtil.clamp(
              turnMotor.getVoltage(turnTorque, module.turnVelocityRadPerSec),
              -NOMINAL_VOLTAGE,
              NOMINAL_VOLTAGE);
      module.turnCurrentAmps = turnCurrent;
    }
  }

  private double computeDriveVoltage(ModuleControl control, double driveVelocityRadPerSec) {
    if (!control.driveClosedLoop) {
      return MathUtil.clamp(control.driveOpenLoopVolts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    }
    double feedbackVolts =
        driveSimP * (control.driveVelocitySetpointRadPerSec - driveVelocityRadPerSec);
    return MathUtil.clamp(
        control.driveFeedforwardVolts + feedbackVolts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
  }

  private double computeTurnVoltage(ModuleControl control, double turnPositionRad) {
    if (!control.turnClosedLoop) {
      return MathUtil.clamp(control.turnOpenLoopVolts, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
    }
    double error = MathUtil.angleModulus(control.turnPositionSetpointRad - turnPositionRad);
    return MathUtil.clamp(turnSimP * error, -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE);
  }

  private static double computeFrictionForce(
      double staticModelForce, double slipVelocity, double maxStaticForce, double maxKineticForce) {
    double slipAbs = Math.abs(slipVelocity);
    if (slipAbs < STATIC_BLEND_VEL && Math.abs(staticModelForce) <= maxStaticForce) {
      return staticModelForce;
    }
    return Math.copySign(maxKineticForce, staticModelForce);
  }

  private static class ModuleControl {
    private boolean driveClosedLoop = true;
    private boolean turnClosedLoop = true;
    private double driveOpenLoopVolts = 0.0;
    private double turnOpenLoopVolts = 0.0;
    private double driveVelocitySetpointRadPerSec = 0.0;
    private double driveFeedforwardVolts = 0.0;
    private double turnPositionSetpointRad = 0.0;
  }

  private static class ModuleState {
    private double drivePositionRad = 0.0;
    private double driveVelocityRadPerSec = 0.0;
    private double turnPositionRad = 0.0;
    private double turnVelocityRadPerSec = 0.0;
    private double driveAppliedVolts = 0.0;
    private double driveCurrentAmps = 0.0;
    private double turnAppliedVolts = 0.0;
    private double turnCurrentAmps = 0.0;
  }

  private static class SimState {
    private double xMeters = 0.0;
    private double yMeters = 0.0;
    private double headingRad = 0.0;
    private double vxRobotMetersPerSec = 0.0;
    private double vyRobotMetersPerSec = 0.0;
    private double omegaRadPerSec = 0.0;
    private final ModuleState[] modules = new ModuleState[MODULE_COUNT];

    private SimState() {
      for (int i = 0; i < MODULE_COUNT; i++) {
        modules[i] = new ModuleState();
      }
    }

    private SimState plus(SimDerivative deriv, double scale) {
      SimState out = new SimState();
      out.xMeters = xMeters + deriv.xMetersPerSec * scale;
      out.yMeters = yMeters + deriv.yMetersPerSec * scale;
      out.headingRad = headingRad + deriv.headingRadPerSec * scale;
      out.vxRobotMetersPerSec = vxRobotMetersPerSec + deriv.vxRobotMetersPerSecSq * scale;
      out.vyRobotMetersPerSec = vyRobotMetersPerSec + deriv.vyRobotMetersPerSecSq * scale;
      out.omegaRadPerSec = omegaRadPerSec + deriv.omegaRadPerSecSq * scale;
      for (int i = 0; i < MODULE_COUNT; i++) {
        out.modules[i].drivePositionRad =
            modules[i].drivePositionRad + deriv.modules[i].drivePositionRadPerSec * scale;
        out.modules[i].driveVelocityRadPerSec =
            modules[i].driveVelocityRadPerSec + deriv.modules[i].driveVelocityRadPerSecSq * scale;
        out.modules[i].turnPositionRad =
            modules[i].turnPositionRad + deriv.modules[i].turnPositionRadPerSec * scale;
        out.modules[i].turnVelocityRadPerSec =
            modules[i].turnVelocityRadPerSec + deriv.modules[i].turnVelocityRadPerSecSq * scale;
      }
      return out;
    }

    private void integrate(
        SimDerivative k1, SimDerivative k2, SimDerivative k3, SimDerivative k4, double dt) {
      xMeters +=
          dt
              / 6.0
              * (k1.xMetersPerSec
                  + 2.0 * k2.xMetersPerSec
                  + 2.0 * k3.xMetersPerSec
                  + k4.xMetersPerSec);
      yMeters +=
          dt
              / 6.0
              * (k1.yMetersPerSec
                  + 2.0 * k2.yMetersPerSec
                  + 2.0 * k3.yMetersPerSec
                  + k4.yMetersPerSec);
      headingRad +=
          dt
              / 6.0
              * (k1.headingRadPerSec
                  + 2.0 * k2.headingRadPerSec
                  + 2.0 * k3.headingRadPerSec
                  + k4.headingRadPerSec);
      vxRobotMetersPerSec +=
          dt
              / 6.0
              * (k1.vxRobotMetersPerSecSq
                  + 2.0 * k2.vxRobotMetersPerSecSq
                  + 2.0 * k3.vxRobotMetersPerSecSq
                  + k4.vxRobotMetersPerSecSq);
      vyRobotMetersPerSec +=
          dt
              / 6.0
              * (k1.vyRobotMetersPerSecSq
                  + 2.0 * k2.vyRobotMetersPerSecSq
                  + 2.0 * k3.vyRobotMetersPerSecSq
                  + k4.vyRobotMetersPerSecSq);
      omegaRadPerSec +=
          dt
              / 6.0
              * (k1.omegaRadPerSecSq
                  + 2.0 * k2.omegaRadPerSecSq
                  + 2.0 * k3.omegaRadPerSecSq
                  + k4.omegaRadPerSecSq);
      for (int i = 0; i < MODULE_COUNT; i++) {
        modules[i].drivePositionRad +=
            dt
                / 6.0
                * (k1.modules[i].drivePositionRadPerSec
                    + 2.0 * k2.modules[i].drivePositionRadPerSec
                    + 2.0 * k3.modules[i].drivePositionRadPerSec
                    + k4.modules[i].drivePositionRadPerSec);
        modules[i].driveVelocityRadPerSec +=
            dt
                / 6.0
                * (k1.modules[i].driveVelocityRadPerSecSq
                    + 2.0 * k2.modules[i].driveVelocityRadPerSecSq
                    + 2.0 * k3.modules[i].driveVelocityRadPerSecSq
                    + k4.modules[i].driveVelocityRadPerSecSq);
        modules[i].turnPositionRad +=
            dt
                / 6.0
                * (k1.modules[i].turnPositionRadPerSec
                    + 2.0 * k2.modules[i].turnPositionRadPerSec
                    + 2.0 * k3.modules[i].turnPositionRadPerSec
                    + k4.modules[i].turnPositionRadPerSec);
        modules[i].turnVelocityRadPerSec +=
            dt
                / 6.0
                * (k1.modules[i].turnVelocityRadPerSecSq
                    + 2.0 * k2.modules[i].turnVelocityRadPerSecSq
                    + 2.0 * k3.modules[i].turnVelocityRadPerSecSq
                    + k4.modules[i].turnVelocityRadPerSecSq);
      }
    }
  }

  private static class SimDerivative {
    private double xMetersPerSec = 0.0;
    private double yMetersPerSec = 0.0;
    private double headingRadPerSec = 0.0;
    private double vxRobotMetersPerSecSq = 0.0;
    private double vyRobotMetersPerSecSq = 0.0;
    private double omegaRadPerSecSq = 0.0;
    private final ModuleDerivative[] modules = new ModuleDerivative[MODULE_COUNT];

    private SimDerivative() {
      for (int i = 0; i < MODULE_COUNT; i++) {
        modules[i] = new ModuleDerivative();
      }
    }
  }

  private static class ModuleDerivative {
    private double drivePositionRadPerSec = 0.0;
    private double driveVelocityRadPerSecSq = 0.0;
    private double turnPositionRadPerSec = 0.0;
    private double turnVelocityRadPerSecSq = 0.0;
  }

  public record ModuleSample(
      double drivePositionRad,
      double driveVelocityRadPerSec,
      double driveAppliedVolts,
      double driveCurrentAmps,
      Rotation2d turnPosition,
      double turnVelocityRadPerSec,
      double turnAppliedVolts,
      double turnCurrentAmps,
      double timestampSec) {}

  public record GyroSample(Rotation2d yaw, double yawVelocityRadPerSec, double timestampSec) {}
}
