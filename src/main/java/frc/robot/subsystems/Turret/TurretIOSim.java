package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretSimMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX44(1), 0.02, TurretConstants.gearing),
          DCMotor.getKrakenX44(1));

  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(0, 0.86);
  private PIDController controller = new PIDController(1, 0, 0);

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(12, 12);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

  private double voltage = 0;

  @Override
  public void setGoal(double angle) {
    if (TurretConstants.canMove) {
      if (angle != goal.position) {
        setpoint =
            new TrapezoidProfile.State(
                turretSimMotor.getAngularPositionRad(),
                turretSimMotor.getAngularVelocityRadPerSec());
        double angleRotations =
            MathUtil.clamp(
                angle, TurretConstants.minAngle / 360.0, TurretConstants.maxAngle / 360.0);
        goal = new TrapezoidProfile.State(Units.rotationsToRadians(angleRotations), 0);
      }
    }
  }

  @Override
  public void setVoltage(double volts) {
    turretSimMotor.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    this.voltage = volts;
  }

  @Override
  public double getPosition() {
    return turretSimMotor.getAngularPositionRotations();
  }

  @Override
  public void lockTurret() {
    TurretConstants.canMove = !TurretConstants.canMove;
  }

  @Override
  public double getGoal() {
    return goal.position;
  }

  @Override
  public boolean readyToShoot() {
    return Math.abs(getPosition() - getGoal()) < TurretConstants.marginOfError;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.goal = Units.radiansToRotations(goal.position);
    inputs.volts = this.voltage;
    inputs.position = turretSimMotor.getAngularPositionRotations();
    inputs.velocity = turretSimMotor.getAngularVelocityRPM() / 60.0;
    inputs.current = turretSimMotor.getCurrentDrawAmps();

    setpoint = profile.calculate(0.02, setpoint, goal);
    setVoltage(
        ffmodel.calculate(setpoint.velocity)
            + controller.calculate(turretSimMotor.getAngularPositionRad(), setpoint.position));

    turretSimMotor.update(0.02);
  }
}
