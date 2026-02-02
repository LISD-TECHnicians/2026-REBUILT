package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public class RobotConstants {
        public class CTREConstants {
            public static double kKrakenX60MaxRPM = 6000.0; // non FOC mode max
            public static double kKrakenX60StallTorque = 7.09;
            public static double kKrakenX60MomentInertia = .000629; // blame Gemini for this one if its off 
            public static double kBatterySupplyVolts = 12.0;
            
            public static AngularAcceleration 
                    kKrakenX60MaxRadsSecondSecond 
                    = Units.RadiansPerSecondPerSecond.of
                    (kKrakenX60StallTorque / kKrakenX60MomentInertia); // estimation of max theoretical 
            public static AngularVelocity 
                    kKrakenX60MaxRadsSecond 
                    = Units.RadiansPerSecond.of((kKrakenX60MaxRPM * 2 * Math.PI) / 2.0);
            public static Current 
                    kKrakenX60MaxCurrent
                     = Units.Amps.of(40); // limited by 40A breaker
            public static Voltage 
                    kKrakenX60OperationVoltage 
                    = Units.Volts.of(kBatterySupplyVolts);
        }

        public class ShooterConstants {
            // Motor IDs
            public static final int kShooterMotor1ID = 10;
            public static final int kShooterMotor2ID = 11;
            public static final int kShooterMotor3ID = 12;
            
            // PWM Channels for servos
            public static final int kPWMChannelLH = 0; // Left Hood servo
            public static final int kPWMChannelRH = 1; // Right Hood servo
            
            // Motor configuration
            public static final InvertedValue kShooterInvertedValue 
                = InvertedValue.CounterClockwise_Positive;
            public static final NeutralModeValue kShooterNeutralModeValue
                = NeutralModeValue.Coast;
            
            // Servo Sets in % values [0 - 1.0]
            public static final double kServoInitPosition = 0.5; // Initial servo position (0.0 - 1.0)
            public static final double kServoMinSet = 0.0;
            public static final double kServoMaxSet = 1.0;
            public static final double kServoPositionTolerance = 0.01; // Tolerance for servo position
            
            // Servo motion limits
            public static final LinearVelocity kMaxLinearServoVelocity 
                = Units.MetersPerSecond.of(0.05); 
            public static final Distance kMaxLinearServoDistance 
                = Units.Meters.of(0.1); 
            
            // Hood geometry (all in meters or degrees)
            public static final Distance kLowerActuatorToPivot 
                = Units.Meters.of(0.15); 
            public static final Distance kUpperActuatorToPivot 
                = Units.Meters.of(0.20); 
            public static final Angle kHoodMinAngle 
                = Units.Degrees.of(10.0); 
            public static final Angle kHoodMaxAngle 
                = Units.Degrees.of(70.0); 
            
            // Linear actuator range (in meters) - Get real measurements here!
            public static final Distance kActuatorMinLength 
                = Units.Meters.of(0.10); 
            public static final Distance kActuatorMaxLength 
                = Units.Meters.of(0.20); 
            public static final Distance kActuatorOffset 
                = Units.Meters.of(0.0); 

            // Shooter physics
            public static final double kRadiusShooterWheel = 0.0508; // 2-inch wheel radius in meters
            public static final double kEffectiveKineticCoef = 0.95; // Energy efficiency coefficient
            
            // Velocity tolerance
            public static final AngularVelocity kVelocityTolerance 
                = Units.RotationsPerSecond.of(2.0);
            
            // Idle speed
            public static final double kIdleShooterPercentage = 0.05; // 5% idle speed to keep wheels warm
        }
    }

    /**
     * Physics constants for projectile motion calculations
     */
    public static class PhysicsConstants {
        // Gravity constant in m/s²
        public static final double kGravity = 9.81; // Earth's gravitational acceleration
        
        // Height difference between shooter and target (in meters)
        // Positive if target is above shooter, negative if below
        public static final double kDeltaHeight = 2.0; // Adjust based on actual field measurements
    }
}

