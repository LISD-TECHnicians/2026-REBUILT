package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.RobotConstants.FeederConstants;

import java.util.List;

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


public class FeederSubsystem extends SubsystemBase{
    
    private TalonFX m_feederMotor;   
    private VelocityVoltage m_feederVelocityRequest 
        = new VelocityVoltage(0);
    private VoltageOut m_feederVoltageRequest 
        = new VoltageOut(0);
    
    public FeederSubsystem() {
        m_feederMotor = new TalonFX(FeederConstants.kFeederMotorID, TunerConstants.kCANBus);
          }

        public void setIndexerMotorSpeed(double setSpeed) {
            m_feederMotor.setControl(
            m_feederVoltageRequest.withOutput
            (Units.Volts.of(setSpeed * CTREConstants.kBatterySupplyVolts.in(Units.Volts))));
          }

        public void setIndexerMotorSpeed(AngularVelocity setSpeed) {
            m_feederMotor.setControl(
            m_feederVelocityRequest.withVelocity
            (Units.RadiansPerSecond.of(setSpeed.in(Units.RadiansPerSecond))));
          }

        public void stopIndexer() {
        m_feederMotor.set(0);
          }

        @Override
        public void periodic() {
        //  
          }
}