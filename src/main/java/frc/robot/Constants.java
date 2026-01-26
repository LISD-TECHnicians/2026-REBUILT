package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

        public class IntakeConstants {
            public static int kIntakeMotorID = 0;
            public static int kPivotMotorID = 1;
            public static double kIntakeSpeedRunCoef = .50; // %'s
            public static double kPivotSpeedRunCoef = .10; 
            public static double kPivotMotorGearReduction = 1.0;
            public static double kPivotDeployVelocityCoef = .5;
            public static double kPivotDeployAccelerationCoef = .8;
            public static double kPivotDeployJerk = 200;
            public static double kPivotHomeVelocityCoef = .8;
            public static double kPivotHomeAccelerationCoef = 1.0;
            public static double kPivotHomeJerk = 200;
            public static double kPivotIndexingVelocityCoef = .7;
            public static double kPivotIndexingAccelerationCoef = .8;
            public static double kPivotIndexingJerk = 200;
            public static double kPivotOcillateVelocityCoef = .2;
            public static double kPivotOcillateAccelerationCoef = .4;
            public static double kPivotOcillateJerk = 300;
            public static double kIntakeSlot0KP = 1.0;
            public static double kIntakeSlot0KI = 0.0;
            public static double kIntakeSlot0KD = 1.0;
            public static double kPivotSlot0KP = 1.0;
            public static double kPivotSlot0KI = 0.0;
            public static double kPivotSlot0KD = 0.0;
            public static double kPivotSlot0KG = 0.1;

            public static Angle kPivotDegreesTolerance = Units.Degrees.of(5);
            public static InvertedValue kIntakeInvertedValue 
                = InvertedValue.CounterClockwise_Positive ;
            public static InvertedValue kPivotInvertedValue 
                = InvertedValue.CounterClockwise_Positive;
            public static NeutralModeValue kIntakeNeutralModeValue = 
                NeutralModeValue.Coast;
            public static NeutralModeValue kPivotNeutralModeValue = 
                NeutralModeValue.Brake;
            public static double kPivotAccelrationMotionCoef = .25;
            public static AngularVelocity kIntakeRunVelocity = 
                CTREConstants.kKrakenX60MaxRadsSecond.times(kIntakeSpeedRunCoef);
            public static AngularVelocity kPivotRunVelocity = 
                CTREConstants.kKrakenX60MaxRadsSecond.times(kPivotSpeedRunCoef);
            public static AngularAcceleration kPivotRunAcceleration = 
                CTREConstants.kKrakenX60MaxRadsSecondSecond.times(kPivotAccelrationMotionCoef);
            public static double kPivotRunJerk = 0.0; // rotations/s^3 - tune as needed
        }
    }
}
