package frc.robot.subsystems;

import java.util.List;

import javax.print.attribute.standard.Media;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
//import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PhysicsConstants;
import frc.robot.Constants.RobotConstants.CTREConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AimHelper;
import frc.robot.util.ShooterHelper;

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
    private final MedianFilter m_tagentialVelocityFilter;
    private final MedianFilter m_servoSetFilter;
    private double m_tangentialVelocity;
    private double m_discriminant;
    private double m_shootingDistance;
    private double m_currentServoPosition;
    private double m_targetServoPositon;
    private ShooterHelper m_helperValues;
    

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
        m_tagentialVelocityFilter = new MedianFilter(100);
        m_servoSetFilter = new MedianFilter(20);
        m_currentServoPosition = ShooterConstants.kServoInitPosition;
        m_targetServoPositon = ShooterConstants.kServoInitPosition;
        m_helperValues = new ShooterHelper(0.0, 0.0);
        
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
            .withVoltage(
                new VoltageConfigs()
                    .withPeakForwardVoltage(Units.Volts.of(11))
                    .withPeakReverseVoltage(Units.Volts.of(-11))
            )
            .withSlot0(
                new Slot0Configs()
                .withKP(0.5)
                .withKI(2)
                .withKD(0)
                .withKV(0.125) // test this as maxRPS / 12v
            );
            m_centerShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_leftShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_rightShooterMotor.getConfigurator().apply(m_shooterMotorConfig);
            m_indexerShooterMotor.getConfigurator().apply(m_shooterMotorConfig);

        m_hoodServoLH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_hoodServoRH.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    private static final InterpolatingTreeMap <Distance, ShooterHelper> m_shooterMap = 
        new InterpolatingTreeMap<> (
            (startDistance, endDistance, q) -> 
                InverseInterpolator.forDouble()
                    .inverseInterpolate(startDistance.in(Units.Meters), endDistance.in(Units.Meters), q.in(Units.Meters)),
            (startValue, endValue, t) -> 
                new ShooterHelper(
                        Interpolator.forDouble()
                            .interpolate(startValue.m_shooterWheelVelocity, endValue.m_shooterWheelVelocity, t),
                        Interpolator.forDouble()
                            .interpolate(startValue.m_hoodSetValue, endValue.m_hoodSetValue, t)
                )
        );

    static {
        /* 
         * Fill this static reference with example data from successful shots that can be used as a basis for future 
         * predictions of rads/s and servo set value. 
        */
        m_shooterMap.put(Units.Meters.of(1.61), new ShooterHelper(350, 0.30));
        m_shooterMap.put(Units.Meters.of(2.82), new ShooterHelper(375, 0.40));
        m_shooterMap.put(Units.Meters.of(3.47), new ShooterHelper(400, 0.50));
        m_shooterMap.put(Units.Meters.of(4.54), new ShooterHelper(440, 0.60));
        m_shooterMap.put(Units.Meters.of(8.22), new ShooterHelper(450, 0.70));
    }

    public void energize(double distance) {
        m_helperValues = m_shooterMap.get(Units.Meters.of(distance));
        setServo(m_helperValues.m_hoodSetValue); 
        setShooterRadiansSecond(m_helperValues.m_shooterWheelVelocity);
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
        m_targetServoPositon = setValue;
    }

    public void testServo(double setPosition) {
        m_hoodServoLH.setPosition(setPosition);
        m_hoodServoRH.setPosition(setPosition);
    }

    
    public Distance calculateServoLength(Angle theta)
    {
        double thetaRadians = theta.in(Units.Radians); // this should not be needed, test and remove.
        
        // TODO: needs to be measured and set for new servos

        double B = ShooterConstants.kLowerActuatorToPivot.in(Units.Meters);
        double C = ShooterConstants.kUpperActuatorToPivot.in(Units.Meters);
        double sinTheta = Math.sin(thetaRadians);
        double cosTheta = Math.cos(thetaRadians);
        double sqrtTerm = Math.sqrt(Math.pow(C, 2) - Math.pow(B * sinTheta, 2));
        
        double length1 = B * cosTheta + sqrtTerm; 
        double length2 = B * cosTheta - sqrtTerm; // assuming the use of the longer length
        
        return Units.Meters.of(length2); 
    }

    public double calculateServoSet(Distance setDistance) {
        double travelDistance = setDistance.in(Units.Meters); // this should be redundant
        double maxTravel = ShooterConstants.kActuatorMaxLength.in(Units.Meters) 
                         - ShooterConstants.kActuatorMinLength.in(Units.Meters);
        // 100 mm travel by design.

        double setValue = -1 * (travelDistance / maxTravel) / 10;
        setValue = m_servoSetFilter.calculate(setValue);
        //System.out.println("setValue: " +  setValue);
        return MathUtil.clamp(setValue, 
            ShooterConstants.kServoMinSet,
            ShooterConstants.kServoMaxSet);
    }

    public void rotateHood(double targetDistance) {
        Angle targetHoodAngle = getCalculatedPitch(targetDistance); 
        Distance calculatedLength = calculateServoLength(targetHoodAngle); // applies law cosines

        //measure and apply for new linear servos from Hitec
        Distance targetActuatorLength 
            = calculatedLength.minus(ShooterConstants.kActuatorOffset); 
        
        Distance linearSetPosition 
            = targetActuatorLength.minus(ShooterConstants.kActuatorMinLength); // get the portion of the distance for travel
        
        double servoSetValue = calculateServoSet(linearSetPosition); 
        
        setServo(servoSetValue);
    }


    public Angle getCalculatedPitch(double targetDistance) {
        m_shootingDistance = targetDistance;
        m_shootingDistance = (m_shootingDistance == 0) ? 0.1 : m_shootingDistance;
        m_tangentialVelocity = getTangentialVelocity() * ShooterConstants.kEffectiveKineticCoef; // coef is a tune point 
        m_tangentialVelocity = m_tagentialVelocityFilter.calculate(m_tangentialVelocity);
        
        m_discriminant = Math.pow(m_tangentialVelocity, 4) - PhysicsConstants.kGravity 
            * (PhysicsConstants.kGravity * Math.pow(m_shootingDistance, 2) + 2 *
            Math.pow(m_tangentialVelocity, 2) * PhysicsConstants.kDeltaHeight);

        if (m_discriminant < 0) {
            return Units.Radians.of(0.0); // Target unreachable
        }

        double tanTheta1 = (Math.pow(m_tangentialVelocity, 2) - Math.sqrt(m_discriminant)) 
            / (PhysicsConstants.kGravity * m_shootingDistance);
       
         double tanTheta2 = (Math.pow(m_tangentialVelocity, 2) + Math.sqrt(m_discriminant)) 
            / (PhysicsConstants.kGravity * m_shootingDistance);
    
        return Units.Radians.of(tanTheta1); // assuming return of larger angle of 2 thetas for loft. 
    }

    public boolean isIndividualMotorAtSpeed(TalonFX motor) {
        //System.out.println("rads/s" + (motor.getVelocity().getValueAsDouble() * 2 * Math.PI)); // confirm and set constant 
        final boolean isVelocityMode = motor.getAppliedControl().equals(m_velocityRequest);
        final AngularVelocity currentAngularVelocity = motor.getVelocity().getValue();
        final AngularVelocity targetAngularVelocity = m_velocityRequest.getVelocityMeasure();
        return isVelocityMode && currentAngularVelocity.isNear(targetAngularVelocity, ShooterConstants.kVelocityToleranceRot); 
    }

    public boolean shooterAtFireSpeed() {
        return m_shooterMotors.stream().allMatch(this::isIndividualMotorAtSpeed);
    }

    public void setShooterRadiansSecond(double setSpeed) { 
        for (final TalonFX shooterMotor : m_shooterMotors) {
            shooterMotor.setControl(
                m_velocityRequest
                    .withVelocity(Units.RadiansPerSecond.of(setSpeed))
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
        //System.out.println("Target Position: " + m_targetServoPositon);
        //System.out.println("Target Position: " + m_currentServoPosition);
        m_hoodServoLH.set(m_targetServoPositon + ShooterConstants.kLeftServoOffset);
        m_hoodServoRH.set(m_targetServoPositon);
        //setServo(.6);
        
        // Keep shooter at idle speed and hood at init position when not commanded
        if (getCurrentCommand() == null) {
            setShooterPercentage(ShooterConstants.kIdleShooterPercentage); 
            setServo(ShooterConstants.kServoInitPosition);
        } 
    }
}


