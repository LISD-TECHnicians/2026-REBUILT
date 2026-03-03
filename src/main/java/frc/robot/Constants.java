
package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

public class Constants {
    public class RobotConstants {
        public class CTREConstants {
            /** 
             * Setup for Field Oriented Control Configuration on KrakenX60 Motors
             * Only includes constants actually used by Intake subsystem
            */
             
            public static final Voltage kBatterySupplyVolts 
                 = Units.Volts.of(12.0);
            
            public static final AngularVelocity kKrakenX60MaxRadsSecond 
                    = Units.RadiansPerSecond.of((96.67 * 2 * Math.PI)); // 96.67 RPS max
            
            public static final AngularAcceleration kKrakenX60MaxRadsSecondSecond 
                = Units.RadiansPerSecondPerSecond.of(11267.0); // Pre-calculated from stall torque/inertia

            public static final AngularVelocity kKrakenX60MaxRPS
                    = Units.RotationsPerSecond.of((96.67)); // 96.67 RPS max

            public static final String kCANBus = null;

        }

    public class FieldConstants {
        public static final Translation2d kBlueHubTranslation2d 
            = new Translation2d(Units.Meters.of(4.625467), Units.Meters.of(4.034663));
        
        public static final Translation2d kRedHubTranslation2d 
            = new Translation2d(Units.Meters.of(11.915521), Units.Meters.of(4.034663));
        }
    
        public class FeederConstants {
            public static final int kFeederMotorID = 9;
            public static final double kFeederMotorSpeed = -0.6;
            public static final double kReverseFeederMotorSpeed = 1.0;
        }

        
        public class IntakeConstants {
            // Motor IDs
            public static final int kIntakeMotorID = 10;
            public static final int kPivotMotorID = 22;
            
            // Speed coefficients
            public static final double kIntakeSpeedRunCoef = 1.0; 
            public static final double kPivotSpeedRunCoef = .02; 
            
            // Pivot motor configuration
            public static final double kPivotMotorGearReduction = 1.0; // update based on team's gearbox selection
            
            public static final double kPivotDeployVelocityCoef = .5;
            public static final double kPivotDeployAccelerationCoef = .8;
            public static final double kPivotDeployJerk = 200;
            //public static final double kPivotDeployVoltage = 0;
            
            public static final double kPivotHomeVelocityCoef = .8;
            public static final double kPivotHomeAccelerationCoef = 1.0;
            public static final double kPivotHomeJerk = 200; 
            //public static final double kPivotHomeVoltage = 0;

            public static final double kPivotIndexingVelocityCoef = .7;
            public static final double kPivotIndexingAccelerationCoef = .8;
            public static final double kPivotIndexingJerk = 200;
            //public static final double kPivotIndexingVoltage = 0;
            
            public static final double kPivotOscillateJerk = 300;
            public static final double kPivotOscillateVelocityCoef = .8;
            public static final double kPivotOscillateAccelerationCoef = 1.0;
            
            public static final double kPivotSlot0KP = 1.0;
            public static final double kPivotSlot0KI = 0.0;
            public static final double kPivotSlot0KD = 0.0;
            public static final double kPivotSlot0KG = 0.1; // Tune based on weight of final intake design

            public static final NeutralModeValue kPivotNeutralModeValue = 
                NeutralModeValue.Brake;

            public static final InvertedValue kPivotInvertedValue 
                = InvertedValue.CounterClockwise_Positive;

            // Tolerances
            public static final Angle kPivotDegreesTolerance 
                = Units.Degrees.of(5);
            
            // Intake motor configuration
            public static final InvertedValue kIntakeInvertedValue 
                = InvertedValue.CounterClockwise_Positive ;
        
            public static final NeutralModeValue kIntakeNeutralModeValue = 
                NeutralModeValue.Coast;
                
            // Fixed pivot motion 
            public static final double kPivotAccelrationMotionCoef = .25;
            
            public static final AngularVelocity kIntakeRunVelocity = 
                CTREConstants.kKrakenX60MaxRPS.times(kIntakeSpeedRunCoef);
            
            public static final AngularVelocity kPivotRunVelocity = 
                CTREConstants.kKrakenX60MaxRPS.times(kPivotSpeedRunCoef);
        
            public static final AngularAcceleration kPivotRunAcceleration = 
                CTREConstants.kKrakenX60MaxRadsSecondSecond.times(kPivotAccelrationMotionCoef);
        }

        public class ShooterConstants {
            // Motor IDs
            public static final int kCenterShooterMotorID = 12;
            public static final int kLeftShooterMotorID = 13; // Directions relative to an overhead view
            public static final int kRightShooterMotorID = 14;
            public static final int kShooterMotorIndexerID = 15;
            
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
            
            // Shooter Motor Sets in % values
            public static final double kMinShooterMotorPercentage = .20;

            // Shooter Motor sets it max rads per seconds
            public static final AngularVelocity kMaxShooterRadsPerSeconds
                = Units.RadiansPerSecond.of(550);

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

            // TODO: Measure angle of ball leaving assembly with fixed approach.
            public static final Angle kHoodFixedAngle // 1st Comp Fixed hood angle
                = Units.Degrees.of(35);
            
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
            
            // Shooter Velocities and magnitudes
            public static final double kMaximumRotationsMagnitude = 91.5; // Tune this Value for adjustments 
            public static final double kMaximumRotationsAcceptableMagnitude = 1.0; // 100% of available motor speed is used 
             public static final double kMinimumRotationsAcceptableMagnitude = 59.68;

            public static final AngularVelocity kVelocityMaximumPossibleRot 
                = Units.RotationsPerSecond.of(kMaximumRotationsMagnitude);

            public static final AngularVelocity kVelocityMaximumPossibleRads
                = Units.RadiansPerSecond.of(kVelocityMaximumPossibleRot.in(Units.RadiansPerSecond));

            public static final AngularVelocity kVelocityMaximumAcceptableRot 
                = Units.RotationsPerSecond.of(kVelocityMaximumPossibleRot.times
                (kMaximumRotationsAcceptableMagnitude).in(
                Units.RotationsPerSecond));
            // refactor this
            public static final AngularVelocity kVelocityMaximumAcceptableRads
                = Units.RotationsPerSecond.of(kVelocityMaximumAcceptableRot.times
                (kMaximumRotationsAcceptableMagnitude).in(
                Units.RadiansPerSecond));

            public static final AngularVelocity kVelocityMimimumAcceptableRads
                = Units.RotationsPerSecond.of(kVelocityMaximumAcceptableRot.times
                (kMinimumRotationsAcceptableMagnitude).in(
                Units.RadiansPerSecond));

            public static final AngularVelocity kVelocityToleranceRot 
                = Units.RotationsPerSecond.of(4.75); // 10% of max --> tune and test 

            
            // Idle speed
            public static final double kIdleShooterPercentage = .15; // 15% idle speed to prevent dead starting at ramp up
        }

        public class ClimbConstants {
            public static final int kClimbMotorID = 16;
        }
    
        public class PathPlannerConstants {
            //public static final RobotConfig robotConfig = null;
        }

        public class DriverConstants {
            public static final int kDriveControllerPort = 0;
            public static final int kOperaterControllerPort = 1;
            public static final double kDriveControllerDeadband = .15; 

            //public static final double DEADBAND = 0.15; 
            //public static final double DEBOUNCE_TIME = 0.2;

            //public static final double kNominalVoltage = 11.0; Check if this is in the TunerConstants
        }

        public static class PhysicsConstants {
        // Gravity constant in m/s²
        public static final double kGravity = 9.81; // Earth's gravitational acceleration
        
        // Height difference between shooter and target (in meters)
        // Positive if target is above shooter, negative if below
        public static final double kDeltaHeight = 2.0; // Adjust based on actual field measurements
}
}
}