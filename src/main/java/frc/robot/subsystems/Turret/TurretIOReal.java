package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOReal implements TurretIO{
  private final TalonFX turretMotor = new TalonFX(TurretConstants.CAN_ID);

  private TalonFXConfiguration turretConfig = new TalonFXConfiguration();

  private final StatusSignal<Current> current = turretMotor.getStatorCurrent();
  private final StatusSignal<Voltage> voltage = turretMotor.getMotorVoltage();
  private final StatusSignal<AngularVelocity> velocity = turretMotor.getVelocity();

  private MotionMagicVoltage control = new MotionMagicVoltage(0);
  private MotionMagicConfigs controlConfig = new MotionMagicConfigs();

    public TurretIOReal() {
      turretConfig.Slot0.kA = TurretConstants.kA;
      turretConfig.Slot0.kV = TurretConstants.kV;
      turretConfig.Slot0.kS = TurretConstants.kS;
      turretConfig.Slot0.kP = TurretConstants.kP;
      turretConfig.Slot0.kI = TurretConstants.kI;
      turretConfig.Slot0.kD = TurretConstants.kD;

      turretConfig.CurrentLimits.StatorCurrentLimit = 60; 
      turretConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
      
      turretMotor.getConfigurator().apply(turretConfig);

      controlConfig.MotionMagicAcceleration = TurretConstants.maxAcceleration;
      controlConfig.MotionMagicCruiseVelocity= TurretConstants.maxVelocity;
      controlConfig.MotionMagicJerk = TurretConstants.maxJerk; 

      turretMotor.getConfigurator().apply(controlConfig);
      
    }

    @Override 
    public double getPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override 
    public void updateMotionProfile() {
        
    }

    @Override 
    public void setGoal(double goal) {
      turretMotor.setControl(control.withPosition(goal));
    }

 
}
