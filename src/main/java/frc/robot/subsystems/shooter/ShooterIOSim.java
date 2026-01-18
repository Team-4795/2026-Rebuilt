package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class ShooterIOSim implements ShooterIO {
  TalonFX topMotor = new TalonFX(ShooterConstants.TOP_CAN_ID);
  TalonFX bottomMotor = new TalonFX(ShooterConstants.BOTTOM_CAN_ID);
  ShooterIOReal real = new ShooterIOReal();
  TalonFXConfiguration topConfig = new TalonFXConfiguration();
  TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
  TalonFXSimState simTopMotor;
  TalonFXSimState simBottomMotor;
  double topVoltage = 0;
  double bottomVoltage = 0;

  private final StatusSignal<AngularVelocity> topRPM = topMotor.getVelocity();
  private final StatusSignal<AngularVelocity> bottomRPM = bottomMotor.getVelocity();
  private final StatusSignal<Current> topCurrent = topMotor.getTorqueCurrent();
  private final StatusSignal<Current> bottomCurrent = bottomMotor.getTorqueCurrent();

  public ShooterIOSim() {
    var topConfig = real.config(ShooterConstants.kV);
    var bottomConfig = real.config(ShooterConstants.kV);

    BaseStatusSignal.setUpdateFrequencyForAll(50, topRPM, bottomRPM, topCurrent, bottomCurrent);

    topMotor.optimizeBusUtilization(1.0);
    bottomMotor.optimizeBusUtilization(1.0);

    bottomMotor.clearStickyFaults();
    topMotor.clearStickyFaults();

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);

    simTopMotor = topMotor.getSimState();
    simBottomMotor = bottomMotor.getSimState();
  }

  @Override
  public void setVoltageTop(double voltage) {
    simTopMotor.setSupplyVoltage(voltage);
  }

  @Override
  public void setVoltageBottom(double voltage) {
    simBottomMotor.setSupplyVoltage(voltage);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}
}
