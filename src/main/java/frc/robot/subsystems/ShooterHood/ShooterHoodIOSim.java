package frc.robot.subsystems.ShooterHood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterHoodIOSim implements ShooterHoodIO {
  private final DCMotorSim shooterHoodSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.02, 30),
          DCMotor.getKrakenX44(1));

  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(0, 0.43, 0);
  private PIDController controller = new PIDController(1, 0, 0);

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ShooterHoodConstants.maxVelocity, ShooterHoodConstants.maxAcceleration);
  private TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);

  private double voltage = 0;

  @Override
  public double getPosition() {
    return shooterHoodSim.getAngularPositionRotations();
  }

  @Override
  public void setVoltage(double volts) {
    shooterHoodSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    this.voltage = volts;
  }

  @Override
  public void setGoal(double angle) {
    if (angle != goal.position) {
      setpoint =
          new TrapezoidProfile.State(
              shooterHoodSim.getAngularPositionRad(), shooterHoodSim.getAngularVelocityRadPerSec());
      double angleRotations =
          MathUtil.clamp(angle, ShooterHoodConstants.minAngle, ShooterHoodConstants.maxAngle);
      goal = new TrapezoidProfile.State(Units.rotationsToRadians(angleRotations), 0);
    }
  }

  @Override
  public void zero() {
    shooterHoodSim.setAngle(0);
  }

  @Override
  public boolean readyToShoot() {
    return Math.abs(getPosition() - getGoal()) < ShooterHoodConstants.marginOfError;
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    inputs.goalRotations = Units.radiansToRotations(goal.position);
    inputs.voltage = this.voltage;
    inputs.position = shooterHoodSim.getAngularPositionRotations();
    inputs.velocityRPS = shooterHoodSim.getAngularVelocityRPM() / 60.0;
    inputs.current = shooterHoodSim.getCurrentDrawAmps();

    setpoint = profile.calculate(0.02, setpoint, goal);
    setVoltage(
        ffmodel.calculate(setpoint.velocity)
            + controller.calculate(shooterHoodSim.getAngularPositionRad(), setpoint.position));

    shooterHoodSim.update(0.02);
  }
}
