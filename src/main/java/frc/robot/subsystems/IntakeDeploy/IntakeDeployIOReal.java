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
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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

  LoggedTunableNumber KP = new LoggedTunableNumber("IntakeDeploy/KP", IntakeDeployConstants.kP);
  LoggedTunableNumber KI = new LoggedTunableNumber("IntakeDeploy/KI", IntakeDeployConstants.kI);
  LoggedTunableNumber KD = new LoggedTunableNumber("IntakeDeploy/KD", IntakeDeployConstants.kD);

  LoggedTunableNumber KS = new LoggedTunableNumber("IntakeDeploy/KS", IntakeDeployConstants.kS);
  LoggedTunableNumber KG = new LoggedTunableNumber("IntakeDeploy/KG", IntakeDeployConstants.kG);
  LoggedTunableNumber KV = new LoggedTunableNumber("IntakeDeploy/KV", IntakeDeployConstants.kV);

  private ArmFeedforward ffmodel = new ArmFeedforward(KS.get(), KG.get(), KV.get());
  private PIDController controller = new PIDController(KP.get(), KI.get(), KD.get());

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          IntakeDeployConstants.MAX_VELOCITY, IntakeDeployConstants.MAX_ACCELERATION);
  private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double deployVolts = 0.0;
  private double PIDVolts = 0.0;
  private double FFVolts = 0.0;

  public IntakeDeployIOReal() {
    // Deploy motor config
    intakeDeployMotorA.clearFaults();
    intakeDeployMotorB.clearFaults();

    deployConfigA.smartCurrentLimit(IntakeDeployConstants.CURRENT_LIMIT);
    deployConfigA.idleMode(IdleMode.kBrake);

    deployConfigA.encoder.positionConversionFactor(1.0 / IntakeDeployConstants.GEARING_DEPLOY);
    deployConfigA.encoder.velocityConversionFactor(
        1.0 / IntakeDeployConstants.GEARING_DEPLOY / 60.0);
    deployConfigA.encoder.quadratureMeasurementPeriod(20);

    deployConfigA.softLimit.forwardSoftLimitEnabled(false);
    deployConfigA.softLimit.reverseSoftLimitEnabled(false);
    deployConfigA.softLimit.forwardSoftLimit(IntakeDeployConstants.deployMaxAngle);
    deployConfigA.softLimit.reverseSoftLimit(IntakeDeployConstants.deployMinAngle);

    deployConfigA.voltageCompensation(12.0);
    deployConfigA.inverted(true);

    deployConfigB.apply(deployConfigA);
    deployConfigB.follow(intakeDeployMotorA.getDeviceId(), true);

    intakeDeployMotorA.configure(
        deployConfigA, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeDeployMotorB.configure(
        deployConfigB, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    zero();

    goal.position = encoderA.getPosition();
    goal.velocity = 0;
  }

  @Override
  public void setGoal(double angle) {
    if (angle != goal.position) {
      setpoint = new TrapezoidProfile.State(encoderA.getPosition(), encoderA.getVelocity());
      goal = new TrapezoidProfile.State(angle, 0);
    }
  }

  @Override
  public void zero() {
    encoderA.setPosition(IntakeDeployConstants.stowPosition);
    encoderB.setPosition(IntakeDeployConstants.stowPosition);
  }

  @Override
  public void updateMotionProfile() {
    // ffmodel = new ArmFeedforward(KS.get(), KG.get(), KV.get());
    // controller = new PIDController(KP.get(), KI.get(), KD.get());

    setpoint = profile.calculate(0.02, setpoint, goal);
    FFVolts =
        ffmodel.calculate(
            Units.rotationsToRadians(encoderA.getPosition()),
            Units.rotationsPerMinuteToRadiansPerSecond(setpoint.velocity * 60));
    PIDVolts = controller.calculate(encoderA.getPosition(), setpoint.position);

    setVoltage(FFVolts + PIDVolts);
  }

  @Override
  public void setVoltage(double v) {
    deployVolts = v;
    intakeDeployMotorA.setVoltage(deployVolts);
  }

  @Override
  public double getPosition() {
    return encoderA.getPosition();
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    ffmodel = new ArmFeedforward(KS.get(), KG.get(), KV.get());
    controller = new PIDController(KP.get(), KI.get(), KD.get());

    inputs.deployMotorVoltage = deployVolts;

    inputs.deployMotorPositionA = encoderA.getPosition();
    inputs.deployMotorVelocityA = encoderA.getVelocity();

    inputs.deployMotorPositionB = encoderB.getPosition();
    inputs.deployMotorVelocityB = encoderB.getVelocity();

    inputs.deployMotorGoal = goal.position;
    inputs.deployMotorSetpoint = setpoint.position;

    inputs.setpointVelocity = setpoint.velocity;
    inputs.setpointPosition = setpoint.position;

    inputs.currentA = intakeDeployMotorA.getOutputCurrent();
    inputs.currentB = intakeDeployMotorB.getOutputCurrent();

    Logger.recordOutput("Intake Deploy/PID Volts", PIDVolts);
    Logger.recordOutput("Intake Deploy/FF Volts", FFVolts);
  }
}
