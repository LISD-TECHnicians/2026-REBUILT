package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.RobotConstants.FeederConstants;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class FeederSubsystem extends SubsystemBase{
    
    private TalonFX m_feederMotor;   
    private VelocityVoltage m_feederVelocityRequest 
        = new VelocityVoltage(0);
    private VoltageOut m_feederVoltageRequest 
        = new VoltageOut(0);
    private TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Units.Amps.of(120))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Units.Amps.of(20))
        .withSupplyCurrentLimitEnable(true));
    
    public FeederSubsystem() {
      m_feederMotor = new TalonFX(FeederConstants.kFeederMotorID, TunerConstants.kCANBus);  
      m_feederMotor.getConfigurator().apply(feederMotorConfig);
    }

    public void setIndexerMotorSpeed(double setSpeed) {
      m_feederMotor.setControl(m_feederVoltageRequest.withOutput
          (Units.Volts.of(setSpeed * CTREConstants.kBatterySupplyVolts.in(Units.Volts))));
    }

    public void setIndexerMotorSpeed(AngularVelocity setSpeed) {
      m_feederMotor.setControl(m_feederVelocityRequest.withVelocity
          (Units.RadiansPerSecond.of(setSpeed.in(Units.RadiansPerSecond))));
    }

    public void stopIndexer() {
      m_feederMotor.set(0);
    }

    @Override
    public void periodic() {}
}