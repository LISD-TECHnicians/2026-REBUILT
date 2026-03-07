package frc.robot.util;

import edu.wpi.first.units.Units;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.RobotConstants.FieldConstants;
import frc.robot.Constants.RobotConstants.PhysicsConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AimHelper {

    public static Rotation2d getRotationToHub(
        Pose2d robotPose,
        Translation2d desiredHubPosition,
        Rotation2d operatorForwardPosition) {
        var alliance = DriverStation.getAlliance();
        double degreeOffset = 0; 
        final Translation2d robotPosition = robotPose.getTranslation();
        final Rotation2d hubRotationBlueAlliance 
            = desiredHubPosition.minus(robotPosition).getAngle();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            degreeOffset = 180;
        }
        final Rotation2d hubRotationOperator 
            = hubRotationBlueAlliance.rotateBy(Rotation2d.fromDegrees(degreeOffset));
        return hubRotationOperator;
    }

    public static Translation2d getHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.kBlueHubTranslation2d;
        }
        else {return FieldConstants.kRedHubTranslation2d;}
    }

    public Translation2d getRobotPosition(CommandSwerveDrivetrain driveSubsystem) {
        return driveSubsystem.getState().Pose.getTranslation();
    }

    public static AngularVelocity getCalculatedSpeed(double m_shootingDistance) {
        m_shootingDistance = (m_shootingDistance == 0) ? 0.1 : m_shootingDistance; 
        
        double fixedHoodRadians = ShooterConstants.kHoodFixedAngle.in(Units.Radians);
        // may just wish to replace with a constant representing the ideal shooter speed. 
        
        double heightComponent = m_shootingDistance * Math.tan(fixedHoodRadians) 
        - PhysicsConstants.kDeltaHeight;
        

        if (heightComponent < 0) {
            return Units.RadiansPerSecond.of(0.0); // Target unreachable 
        }
        
        double denominator = 2 * Math.pow(Math.cos(fixedHoodRadians), 2) * heightComponent;

        double exitV = Math.sqrt((PhysicsConstants.kGravity * Math.pow(m_shootingDistance, 2)) / denominator);

        double exitRotationalV = exitV / ShooterConstants.kRadiusShooterWheel;

        return Units.RadiansPerSecond.of(exitRotationalV * ShooterConstants.kEffectiveKineticCoef);
    }

    public static boolean aimAcceptable(
        Rotation2d idealCoordinate,
        Rotation2d actualCoordinate, 
        Angle acceptableDeviation) {
            double idealRadians = idealCoordinate.getRadians();
            double idealRadiansConstrained = MathUtil.angleModulus(idealRadians); // -pi to pi constaint.
            
            double actualRadians = actualCoordinate.getRadians();
            double actualRadiansConstrained = MathUtil.angleModulus(actualRadians);
            
            return MathUtil.isNear(
                actualRadiansConstrained, 
                idealRadiansConstrained, 
                acceptableDeviation.in(Units.Radians),
                -Math.PI,
                 Math.PI
                );
    }
}
