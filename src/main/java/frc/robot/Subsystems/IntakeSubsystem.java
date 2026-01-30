package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

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

import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    public enum Position {
        DEPLOYED(0),
        INDEXING(20),
        HOME(100);

        private double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle positionDegrees() {
            return Units.Degrees.of(degrees);
        }
    }

    private TalonFX m_intakeMotor;
    private TalonFX m_pivotMotor;    
    private VelocityVoltage m_intakeVelocityRequest 
        = new VelocityVoltage(0);
    private VoltageOut m_intakeVoltageRequest 
        = new VoltageOut(0);
    private DynamicMotionMagicVoltage m_pivotDynamicMotionMagicRequest 
            = new DynamicMotionMagicVoltage(
            Units.Rotations.of(0),  
            IntakeConstants.kPivotRunVelocity, 
            IntakeConstants.kPivotRunAcceleration); 

    public IntakeSubsystem() {
        m_intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);
        m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID);
        configureMotors();
    }

    public void configureIntakeMotor() {
        TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kIntakeInvertedValue)
                    .withNeutralMode(IntakeConstants.kIntakeNeutralModeValue)
                
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Units.Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
            m_intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    public void configurePivotMotor() {
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(IntakeConstants.kPivotInvertedValue)
                    .withNeutralMode(IntakeConstants.kPivotNeutralModeValue)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Units.Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(IntakeConstants.kPivotMotorGearReduction)
            ) 
            .withSlot0(
                new Slot0Configs()
                    .withKP(IntakeConstants.kPivotSlot0KP)
                    .withKI(IntakeConstants.kPivotSlot0KI)
                    .withKD(IntakeConstants.kPivotSlot0KD)
                    .withKV(CTREConstants.kBatterySupplyVolts.in(Units.Volts)
                    / (CTREConstants.kKrakenX60MaxRadsSecond.in(Units.RotationsPerSecond) 
                    * (IntakeConstants.kPivotMotorGearReduction)))
                    .withKG(IntakeConstants.kPivotSlot0KG)
            );
            m_pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }       

    public void configureMotors() {
        configureIntakeMotor();
        configurePivotMotor();
    }

    public void setPivotPosition(Position position,
        AngularVelocity requestVelocity, 
        AngularAcceleration requestAcceleration,
        double requestJerk) {
        m_pivotDynamicMotionMagicRequest.Velocity 
            = requestVelocity.in(Units.RotationsPerSecond);
        m_pivotDynamicMotionMagicRequest.Acceleration 
            = requestAcceleration.in(Units.RotationsPerSecondPerSecond);
        m_pivotDynamicMotionMagicRequest.Jerk = requestJerk; // rotations per second cubed
        m_pivotMotor.setControl(
            m_pivotDynamicMotionMagicRequest
            .withPosition(position.positionDegrees())
        );
    }

    public void setIntakeMotorSpeed(double setSpeed) {
        m_intakeMotor.setControl(
            m_intakeVoltageRequest
                .withOutput(Units.Volts.of(
                    setSpeed
                    * CTREConstants.kBatterySupplyVolts.in(Units.Volts)
                    )
                )
        );
    }

    public void setIntakeMotorSpeed(AngularVelocity setSpeed) {
        m_intakeMotor.setControl(
            m_intakeVelocityRequest
                .withVelocity(Units.RadiansPerSecond.of(
                    setSpeed.in(Units.RadiansPerSecond)
                    )
                )
        );
    }

    public boolean pivotInPosition() {
        final Angle currentPosition = m_pivotMotor.getPosition().getValue();
        final Angle targetPosition = m_pivotDynamicMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, IntakeConstants.kPivotDegreesTolerance);
    }

    public void stopIntake() {
        m_intakeMotor.set(0);
    }

    public void stopPivot() {
        m_pivotMotor.set(0);
    }

    @Override
    public void periodic() {
        // TODO: Add logging with smartdashboard 
    }
}
