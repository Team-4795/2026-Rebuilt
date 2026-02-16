package frc.robot.subsystems.IntakeDeploy;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class IntakeDeployIOReal implements IntakeDeployIO {
  // two motors to extend intake
  private final SparkFlex intakeDeployMotorA =
      new SparkFlex(IntakeDeployConstants.canIDDeployA, MotorType.kBrushless);
  private final SparkFlex intakeDeployMotorB =
      new SparkFlex(IntakeDeployConstants.canIDDeployB, MotorType.kBrushless);

  private SparkFlexConfig deployConfigA = new SparkFlexConfig();
  private SparkFlexConfig deployConfigB = new SparkFlexConfig();
  private RelativeEncoder encoderA = intakeDeployMotorA.getEncoder();
  private RelativeEncoder encoderB = intakeDeployMotorB.getEncoder();

  private ArmFeedforward ffmodel =
      new ArmFeedforward(
          IntakeDeployConstants.kS, IntakeDeployConstants.kG, IntakeDeployConstants.kV);
  private PIDController controller =
      new PIDController(
          IntakeDeployConstants.kP, IntakeDeployConstants.kI, IntakeDeployConstants.kD);

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          IntakeDeployConstants.MAX_VELOCITY, IntakeDeployConstants.MAX_ACCELERATION);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double deployVolts = 0;

  public IntakeDeployIOReal() {
    // Deploy motor config
    intakeDeployMotorA.clearFaults();
    intakeDeployMotorB.clearFaults();

    deployConfigA.smartCurrentLimit(IntakeDeployConstants.CURRENT_LIMIT);
    deployConfigA.idleMode(IdleMode.kBrake);

    deployConfigA.encoder.positionConversionFactor(IntakeDeployConstants.GEARING_DEPLOY);
    deployConfigA.encoder.velocityConversionFactor(IntakeDeployConstants.GEARING_DEPLOY / 60.0);
    deployConfigA.encoder.quadratureMeasurementPeriod(20);

    deployConfigA.softLimit.forwardSoftLimitEnabled(true);
    deployConfigA.softLimit.reverseSoftLimitEnabled(true);
    deployConfigA.softLimit.forwardSoftLimit(IntakeDeployConstants.deployMaxAngle);
    deployConfigA.softLimit.reverseSoftLimit(IntakeDeployConstants.deployMinAngle);

    deployConfigA.voltageCompensation(12.0);
    deployConfigA.inverted(false);

    deployConfigB.apply(deployConfigA);
    deployConfigB.follow(intakeDeployMotorA.getDeviceId(), true);

    intakeDeployMotorA.configure(
        deployConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeDeployMotorB.configure(
        deployConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setGoal(double angle) {
    if (angle != goal.position) {
      setpoint = new TrapezoidProfile.State(encoderA.getPosition(), encoderA.getVelocity() / 60.0);
      goal = new TrapezoidProfile.State(angle, 0);
    }
  }

  @Override
  public void updateMotionProfile() {
    setpoint = profile.calculate(0.02, setpoint, goal);
    double ffvolts =
        ffmodel.calculate(
            Units.rotationsToRadians(encoderA.getPosition() - IntakeDeployConstants.deployOffset),
            Units.rotationsPerMinuteToRadiansPerSecond(setpoint.velocity * 60.0));
    double pidvolts = controller.calculate(encoderA.getPosition(), setpoint.position);

    setVoltage(ffvolts + pidvolts);
  }

  @Override
  public void setVoltage(double v) {
    deployVolts = v;
    intakeDeployMotorA.setVoltage(deployVolts);
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    inputs.deployMotorVoltage = deployVolts;
    inputs.deployMotorPositionA = encoderA.getPosition();
    inputs.deployMotorVelocityA = encoderA.getVelocity();
    inputs.deployMotorPositionB = encoderB.getPosition();
    inputs.deployMotorVelocityB = encoderB.getVelocity();
    inputs.deployMotorGoal = goal.position;
    inputs.deployMotorSetpoint = setpoint.position;
  }
}
