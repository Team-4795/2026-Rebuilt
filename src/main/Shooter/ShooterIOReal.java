package frc.robot.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOReal implements ShooterIO {
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.topCanID);
  private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.bottomCanID);
  private TalonFXConfigurator config = new TalonFXConfigurator();

  public VelocityVoltage controller = new VelocityVoltage();

  private TalonFXConfiguration config(double kV) {
    var talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Slot0.kP = ShooterConstants.kP;
    talonFXConfig.Slot0.kI = ShooterConstants.kI;
    talonFXConfig.Slot0.kD = ShooterConstants.kD;
    talonFXConfig.Slot0.kS = ShooterConstants.kS;
    talonFXConfig.Slot0.kV = ShooterConstants.kV;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.currentLimit;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Audio.BeepOnBoot = true;

    return talonFXConfig;
  }

  public ShooterIOReal() {
    var topConfig = config(ShooterConstants.kV);
    var bottomConfig = config(ShooterConstants.kV);

    BaseStatusSignal.setUpdateFrequencyForAll(0, null);

    topShooterMotor.optimizeBusUtilization(1.0);
    bottomShooterMotor.optimizeBusUtilization(1.0);


    bottomShooterMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();

    
  }
}
