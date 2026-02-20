package frc.robot.commands;

import java.util.function.DoubleSupplier;

import  edu.wpi.first.units.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AimHelper;
import frc.robot.util.JoystickProfileHelper;
import frc.robot.util.ManualDriveInput;
import frc.robot.generated.TunerConstants;

public class RotationalAimCommand extends Command {
    private Translation2d hubPosition;
    private Rotation2d desiredRotation;
    private final Angle m_allowedDeviation  = Units.Degrees.of(5);
    private final CommandSwerveDrivetrain m_commandSwerveDrivetrain;
    private final SwerveRequest.FieldCentricFacingAngle m_fieldCentricFacingAngle = 
        new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Units.RotationsPerSecond.of(1).times(.005)) // create and pull from Constants.
            .withMaxAbsRotationalRate(Units.RotationsPerSecond.of(1))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(5, 0, 0); // TODO: test and tune this.

    private final JoystickProfileHelper m_profileHelper;

    public RotationalAimCommand(
        CommandSwerveDrivetrain swerveDrivetrain,
        DoubleSupplier yValue,
        DoubleSupplier xValue) 
    {
        this.m_commandSwerveDrivetrain = swerveDrivetrain;
        this.m_profileHelper = new JoystickProfileHelper(yValue, xValue);
        addRequirements(swerveDrivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        hubPosition = AimHelper.getHubPosition();
        desiredRotation = AimHelper.getRotationToHub
        (m_commandSwerveDrivetrain.getState().Pose, 
         hubPosition, 
         desiredRotation);
        final ManualDriveInput input = m_profileHelper.getSmoothedInput();
        m_commandSwerveDrivetrain.setControl
        (
            m_fieldCentricFacingAngle
                .withVelocityX(TunerConstants.kSpeedAt12Volts.times(input.forward))
                .withVelocityY(TunerConstants.kSpeedAt12Volts.times(input.left))
                .withTargetDirection(desiredRotation)
        );
    }           

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
        /* 
        Translation2d hubPosition = AimHelper.getHubPosition();
        Rotation2d targetRotation = AimHelper.getRotationToHub(
            m_commandSwerveDrivetrain.getState().Pose,
            hubPosition,
            m_commandSwerveDrivetrain.getOperatorForwardDirection()
        );
        Rotation2d currentRotation = m_commandSwerveDrivetrain.getState().Pose.getRotation();
        
        return AimHelper.aimAcceptable(targetRotation, currentRotation, m_allowedDeviation);
        */
        }
}
