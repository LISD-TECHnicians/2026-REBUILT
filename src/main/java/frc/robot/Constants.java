package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    public class FieldConstants {
        public static final Translation2d kBlueHubTranslation2d 
            = new Translation2d(Units.Meters.of(4.625467), Units.Meters.of(4.034663));
        
        public static final Translation2d kRedHubTranslation2d 
            = new Translation2d(Units.Meters.of(11.915521), Units.Meters.of(4.034663));
    }
}
