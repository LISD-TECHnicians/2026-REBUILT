package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        }
    
        public class IndexerConstants {
            
            public static final int kIndexerMotorID = 9;
            private static final boolean kIndexerMotorInverted = false;
        }
    }
}
