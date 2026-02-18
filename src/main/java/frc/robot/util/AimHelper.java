package frc.robot.util;

import edu.wpi.first.units.Units;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.FieldConstants;

public class AimHelper {

    public static Rotation2d getRotationToHub(
        Pose2d robotPose,
        Translation2d desiredHubPosition,
        Rotation2d operatorForwardPosition) {
        final Translation2d robotPosition = robotPose.getTranslation();
        final Rotation2d hubRotationBlueAlliance 
            = desiredHubPosition.minus(robotPosition).getAngle();
        final Rotation2d hubRotationOperator 
            = hubRotationBlueAlliance.rotateBy(operatorForwardPosition);
        return hubRotationOperator;
    }

    public static Translation2d getHubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.kBlueHubTranslation2d;
        }
        else {return FieldConstants.kRedHubTranslation2d;}
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
