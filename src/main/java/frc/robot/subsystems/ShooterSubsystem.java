package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PhysicsConstants;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase{

    private final TalonFX m_centerShooterMotor;
    private final TalonFX m_leftShooterMotor;
    private final TalonFX m_rightShooterMotor;
    private final TalonFX m_indexerShooterMotor;
    private final List<TalonFX> m_shooterMotors;
    private final Servo m_hoodServoLH;
    private final Servo m_hoodServoRH;
    private Time m_FPGATimeStamp;
    private final VelocityVoltage m_velocityRequest;
    private final VoltageOut m_indexerVoltageRequest;
    private final VoltageOut m_shooterVoltageRequest;
    private TalonFXConfiguration m_shooterMotorConfig;
    private double m_tangentialVelocity;
    private double m_discriminant;
    private double m_shootingDistance;
    private double m_currentServoPosition;
    private double m_targetServoPositon;
    

    public ShooterSubsystem() {
        m_FPGATimeStamp = Units.Seconds.of(0);
        m_centerShooterMotor = new TalonFX(ShooterConstants.kCenterShooterMotorID, TunerConstants.kCANBus);
        m_leftShooterMotor = new TalonFX(ShooterConstants.kLeftShooterMotorID, TunerConstants.kCANBus);
        m_rightShooterMotor = new TalonFX(ShooterConstants.kRightShooterMotorID, TunerConstants.kCANBus);
        m_indexerShooterMotor = new TalonFX(ShooterConstants.kShooterMotorIndexerID, TunerConstants.kCANBus);
        m_shooterMotors = List.of(m_centerShooterMotor, m_leftShooterMotor, m_rightShooterMotor);
        m_velocityRequest = new VelocityVoltage(0);
        m_shooterVoltageRequest = new VoltageOut(0);
        m_indexerVoltageRequest = new VoltageOut(0);
        m_hoodServoLH = new Servo(ShooterConstants.kPWMChannelLH);
        m_hoodServoRH = new Servo(ShooterConstants.kPWMChannelRH);
        m_currentServoPosition = ShooterConstants.kServoInitPosition;
        m_targetServoPositon = ShooterConstants.kServoInitPosition;
        
        configureMotors();
    }

    public void configureMotors() {
        m_shooterMotorConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(ShooterConstants.kShooterInvertedValue)
                    .withNeutralMode(ShooterConstants.kShooterNeutralModeValue)
                
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Units.Amps.of(120))
                    .withStatorCurrentLimitEnable(false) // testing 
                    .withSupplyCurrentLimit(Units.Amps.of(120))
                    .withSupplyCurrentLimitEnable(false)
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(1)
                .withKI(0)
                .withKD(0)
                .withKV(.10) // test this as maxRPS / 12v
            );
            m_centerShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_leftShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_rightShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_indexerShooterMotor.getConfigurator().apply(m_shooterMotorConfig);

        m_hoodServoLH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_hoodServoRH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    public boolean isPositionWithinTolerance() {
        return MathUtil.isNear(m_targetServoPositon, 
            m_currentServoPosition, 
            ShooterConstants.kServoPositionTolerance);
    }

    public void updateServoPosition() {
        Time currentTime = Units.Seconds.of(Timer.getFPGATimestamp());
        Time deltaTime = currentTime.minus(m_FPGATimeStamp);
        m_FPGATimeStamp = currentTime;

        if (isPositionWithinTolerance()) {
            m_currentServoPosition  = m_targetServoPositon; return; 
        }

        Distance maxServoDistance = 
            ShooterConstants.kMaxLinearServoVelocity.times(deltaTime);
        double maxServoPercentage = maxServoDistance.div(ShooterConstants.kMaxLinearServoDistance).in(Units.Value);
        m_currentServoPosition = m_targetServoPositon > m_currentServoPosition
            ? Math.min(m_targetServoPositon, m_currentServoPosition + maxServoPercentage)
            : Math.max(m_targetServoPositon, m_currentServoPosition - maxServoPercentage);        
    }

    public void setServo(double setValue) {
        m_targetServoPositon = MathUtil.clamp(
                setValue, 
                ShooterConstants.kServoMinSet, 
                ShooterConstants.kServoMaxSet);
    }

    /* 
    public Velocity calculateFixedHoodShooterSpeed() {
        m_shootingDistance = getShootingDistance(); 
        m_shootingDistance = (m_shootingDistance == 0) ? 0.1 : m_shootingDistance; 


    }
    */

    /**
     * Calculates the required linear actuator length for a given hood angle using law of cosines.
     * 
     * Derived formula: A = B * cos(theta) ± sqrt(C² - B² * sin²(theta))
     * Where: A = actuator length (unknown)
     *        B = distance from lower actuator mount to hood pivot
     *        C = distance from upper actuator mount to hood pivot
     *        theta = desired hood angle
     * 
     * @param theta Desired hood angle
     * @return Required actuator length in meters
     */

    public Distance calculateServoLength(Angle theta)
    {
        double thetaRadians = theta.in(Units.Radians); 
        
        double B = ShooterConstants.kLowerActuatorToPivot.in(Units.Meters);
        double C = ShooterConstants.kUpperActuatorToPivot.in(Units.Meters);
        double sinTheta = Math.sin(thetaRadians);
        double cosTheta = Math.cos(thetaRadians);
        double sqrtTerm = Math.sqrt(Math.pow(C, 2) - Math.pow(B * sinTheta, 2));
        
        double length1 = B * cosTheta + sqrtTerm; 
        double length2 = B * cosTheta - sqrtTerm; 
        
        // 
        return Units.Meters.of(length2); 
    }

    public double calculateServoSet(Distance setDistance) {
        double travelDistance = setDistance.in(Units.Meters);
        double maxTravel = ShooterConstants.kActuatorMaxLength.in(Units.Meters) 
                         - ShooterConstants.kActuatorMinLength.in(Units.Meters);
        
        double setValue = travelDistance / maxTravel;
        
        return MathUtil.clamp(setValue, 
            ShooterConstants.kServoMinSet,
            ShooterConstants.kServoMaxSet);
    }

    public void rotateHood() {
        Angle targetHoodAngle = getCalculatedPitch(); 
        Distance calculatedLength = calculateServoLength(targetHoodAngle); // applies law cosines
        Distance targetActuatorLength 
            = calculatedLength.minus(ShooterConstants.kActuatorOffset); 
        Distance linearSetPosition 
            = targetActuatorLength.minus(ShooterConstants.kActuatorMinLength); // get the portion of the distance for travel
        double servoSetValue = calculateServoSet(linearSetPosition); 
        setServo(servoSetValue);
    }

    public Angle getCalculatedPitch() {
        m_shootingDistance = getShootingDistance(); 
        m_shootingDistance = (m_shootingDistance == 0) ? 0.1 : m_shootingDistance; 
        
        /* 
                 may just wish to replace with a constant representing the ideal shooter speed. 
        */ 
        m_tangentialVelocity = getTangentialVelocity() * ShooterConstants.kEffectiveKineticCoef;
        
        m_discriminant = Math.pow(m_tangentialVelocity, 4) - PhysicsConstants.kGravity
             * (PhysicsConstants.kGravity * Math.pow(m_shootingDistance, 2) + 2 * 
                Math.pow(m_tangentialVelocity, 2) * PhysicsConstants.kDeltaHeight);
        
        if (m_discriminant < 0) {
            return Units.Degrees.of(0.0); // Target unreachable 
        }
        
        double tanTheta1 = (Math.pow(m_tangentialVelocity, 2) - Math.sqrt(m_discriminant)) 
                / (PhysicsConstants.kGravity * m_shootingDistance); 

        double tanTheta2 = (Math.pow(m_tangentialVelocity, 2) + Math.sqrt(m_discriminant)) 
            / (PhysicsConstants.kGravity * m_shootingDistance); 

        // Use higher arc (tanTheta2) for shot trajectory
        double thetaRadians = Math.atan(tanTheta2);
        double thetaDegrees = Math.toDegrees(thetaRadians);
        
        thetaDegrees 
            = MathUtil.clamp(thetaDegrees, 
                            ShooterConstants.kHoodMinAngle.in(Units.Degrees), 
                            ShooterConstants.kHoodMaxAngle.in(Units.Degrees));
        
        return Units.Degrees.of(thetaDegrees); 
    }

    public boolean isIndividualMotorAtSpeed(TalonFX motor) {
        System.out.println("rots/s" + motor.getVelocity().getValueAsDouble());
        final boolean isVelocityMode = motor.getAppliedControl().equals(m_velocityRequest);
        final AngularVelocity currentAngularVelocity = motor.getVelocity().getValue();
        final AngularVelocity targetAngularVelocity = m_velocityRequest.getVelocityMeasure();
        return isVelocityMode && currentAngularVelocity.isNear(targetAngularVelocity, ShooterConstants.kVelocityTolerance); 
    }

    public boolean shooterAtFireSpeed() {
        return m_shooterMotors.stream().allMatch(this::isIndividualMotorAtSpeed);
    }

    public void setShooterRadiansSecond(double setSpeed) { 
        for (final TalonFX shooterMotor : m_shooterMotors) {
            shooterMotor.setControl(
                m_velocityRequest
                    .withVelocity(Units.RadiansPerSecond.of(565))
            );
      }
    }


    public void setIndexerMotorPercentage(double setSpeed) {
        m_indexerShooterMotor.setControl(m_shooterVoltageRequest
            .withOutput(CTREConstants.kBatterySupplyVolts.times(setSpeed)));
    }

     public void setShooterPercentage(double setSpeed) {
        for (final TalonFX shooterMotor : m_shooterMotors) {
            shooterMotor.setControl(
                m_shooterVoltageRequest
                    .withOutput(CTREConstants.kBatterySupplyVolts.times(setSpeed))
            );
        }
     }

    public void stopShooterMotors() {
        setShooterPercentage(0.0);
    }

    public void stopIndexerMotor() {
        setIndexerMotorPercentage(0.0);
    }

    public void reverseIndexerMotor(double setSpeed) {
        m_indexerShooterMotor.setControl(m_shooterVoltageRequest
            .withOutput(CTREConstants.kBatterySupplyVolts.times(-setSpeed)));
    }
    
    public double getShootingDistance() {
        return 0.0; // Consider getting this value from a command reference to the subsystem.
    }

    public double getTangentialVelocity() {
        double wheelRPM = m_centerShooterMotor.getVelocity().getValueAsDouble() * 60.0; // Convert rot/s to RPM
        double radiansPerSecond = (wheelRPM * 2 * Math.PI) / 60.0;
        return radiansPerSecond * ShooterConstants.kRadiusShooterWheel;
    }

    @Override
    public void periodic() { 
        // Update servo position with rate limiting
        updateServoPosition();
        
        // Apply the rate-limited position to the physical servos
        m_hoodServoLH.set(m_currentServoPosition);
        m_hoodServoRH.set(m_currentServoPosition);
        
        // Keep shooter at idle speed and hood at init position when not commanded
        if (getCurrentCommand() == null) {
            setShooterPercentage(ShooterConstants.kIdleShooterPercentage); 
            setServo(ShooterConstants.kServoInitPosition);
        } 
    }
}


