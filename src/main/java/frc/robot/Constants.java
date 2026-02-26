package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public class RobotConstants {
        public class CTREConstants {
            /** 
             * Setup for Field Oriented Control Configuration on KrakenX60 Motors
             * Only includes constants actually used by Intake subsystem
            */ 
            public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

            public static final Voltage kBatterySupplyVolts 
                 = Units.Volts.of(12.0);
            
            public static final AngularVelocity kKrakenX60MaxRPS
                    = Units.RotationsPerSecond.of((96.67)); // 96.67 RPS max

            public static final AngularAcceleration kKrakenX60MaxRadsSecondSecond 
                = Units.RadiansPerSecondPerSecond.of(11267.0); // Pre-calculated from stall torque/inertia
        }

        public class IntakeConstants {
            // Motor IDs
            public static final int kIntakeMotorID = 10;
            public static final int kPivotMotorID = 11;
            
            // Speed coefficients
            public static final double kIntakeSpeedRunCoef = .85; 
            public static final double kPivotSpeedRunCoef = .20; 
            
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
    }
}
