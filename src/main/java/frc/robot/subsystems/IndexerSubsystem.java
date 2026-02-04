package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.IndexerConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs; 
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


public class IndexerSubsystem extends SubsystemBase{
    
    private TalonFX m_indexerMotor;   
    private VelocityVoltage m_indexerVelocityRequest 
        = new VelocityVoltage(0);
    private VoltageOut m_indexerVoltageRequest 
        = new VoltageOut(0);
    
    public IndexerSubsystem() {
        m_indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorID, TunerConstants.kCANBus);
          }

        public void setIndexerMotorSpeed(double setSpeed) {
            m_indexerMotor.setControl(
            m_indexerVoltageRequest.withOutput
            (Units.Volts.of(setSpeed * CTREConstants.kBatterySupplyVolts.in(Units.Volts))));
          }

        public void setIndexerMotorSpeed(AngularVelocity setSpeed) {
            m_indexerMotor.setControl(
            m_indexerVelocityRequest.withVelocity
            (Units.RadiansPerSecond.of(setSpeed.in(Units.RadiansPerSecond))));
          }

        public void stopIndexer() {
        m_indexerMotor.set(0);
          }

        @Override
        public void periodic() {
        // TODO: Add logging with smartdashboard 
          }
}