package frc.robot;

public class Constants {
    public static class FieldConstants {
        public static double kHeightAprilTag = 0.0; 
    }

    public static class OperatorConstants {}

    public static class PathPlannerConstants {}    

    public static class PhysicsConstants {
        public static double kGravity = -9.81; // in m/s^2
    }

    public static class RobotConstants {

        public static final double kNeoMaxRpm = 5676.0;
        public static final double kNeoMotorSetMax = 1.0;
        public static final double kNeoMotorSetMin = -1.0;

        public static class DriveTrainConstants {}

        public static class IntakeConstants {
            public static final int kIntakeMotorID = 3;
            public static final int kPivotMotorID = 4;
            public static final double kPivotPositionVarience = 5.0; // degrees
            public static final double kPivotAngleStored = 0.0;
            public static final double kPivotAngleActive = 90.0;
            public static final double kPivotAngleOut = 80.0;
            public static final double kPivotGearRatio = 1.0;
            public static final double kPivotMaxDegrees = 45;
            public static final double kPivotDegreesSecond = 90;
            public static final double kDegreesSecondVariation = 5.0; 
            public static final double kPivotDegreesSecondSquare = 180; 
            public static final double kActiveIntakeSpeed = .25; 
            public static final double kIntakeProportion = 1.0; 
            public static final double kIntakeIntegration = 0.0;
            public static final double kIntakeDerivative = 0.0; 
            public static final double kPivotProportion = 1.0; 
            public static final double kPivotIntegration = 0.0;
            public static final double kPivotDerivative = 0.0; 
        }
        public static class LimelightConstants {}

        public static class ShooterConstants {
            public static final int kPitchMotorID = 0;
            public static final int kShooterMotorID = 1;
            public static final double kShooterWheelSpeedFire = .25;
            public static final double kShooterWheelSpeedIdle = kShooterWheelSpeedFire / 2;
            public static final double kEffectiveShooterHeight = .75; 
            public static final double kRadiusShooterWheel = 0.25; // get in meters
            public static final double kPitchGearRatio = 1.0;
            public static final double kPitchPlaneCorrection = 0.0; // geometry offset 
            public static final double kPitchRestAngle = 45.0; // calculate from average use case.
            public static final double kPitchCorrection = 0.0; // Tune via trial and error with physical bot.
            public static final double kEffectiveKineticCoef = 1.0; 
            public static final double kPitchProportion = 1.0; 
            public static final double kPitchIntegration = 0.0;
            public static final double kPitchDerivative = 0.0; 
            public static final double kShooterProportion = 1.0; 
            public static final double kShooterIntegration = 0.0;
            public static final double kShooterDerivative = 0.0; 
            public static final double[] kPitchRangeDegrees= {0, 30};
        }
    }

     
}
