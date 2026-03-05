// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants.DriverConstants;
import frc.robot.Constants.RobotConstants.FeederConstants;
import frc.robot.Constants.RobotConstants.ClimbConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.RotationalAimCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.UpdateVisionMeasurementCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;
import frc.robot.subsystems.ClimbSubsystem;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double slowSpeedCoef = .60;
    private double slowSpeedCoefRotational = .60;
    
    private double speedCrabWalkCoef = .75;

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();

    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle m_crabWalkRequest 
        = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(Rotation2d.fromDegrees(45));

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandPS4Controller driveController = new CommandPS4Controller(DriverConstants.kDriveControllerPort);
    private final CommandPS4Controller operatorController = new CommandPS4Controller(DriverConstants.kOperaterControllerPort);
    //private final Trigger brakeTrigger = new Trigger(() -> false);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    ;

    public final LimelightSubsystem limelight = new LimelightSubsystem("limelight-front");
    private final Trigger updateVisionTrigger = new Trigger(() -> LimelightHelpers.getTV("limelight-front"));
    private final UpdateVisionMeasurementCommand updateVisionOdometry = new UpdateVisionMeasurementCommand(drivetrain, limelight);
    private final SendableChooser<Command> autoChooser;
    

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        registerNamedCommands();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * .60) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed * .60) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    .withDeadband(DriverConstants.kDriveControllerDeadband)
                    )
        );

        updateVisionTrigger.whileTrue(updateVisionOdometry);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        driveController.cross().toggleOnTrue(drivetrain.applyRequest(() -> brake));
            
        driveController.circle().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        driveController.L1().whileTrue(drivetrain.applyRequest(() ->  
            drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * 0.90)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed * 0.90)
            .withRotationalRate(-driveController.getRightX())
            ));

        /*driveController.povDown().whileTrue(drivetrain.applyRequest(() ->  
            m_crabWalkRequest.withVelocityX(driveController.getLeftY() * MaxSpeed * slowSpeedCoef)
            .withVelocityY(driveController.getLeftX() * MaxSpeed * slowSpeedCoef)
            ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.share().and(driveController.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.share().and(driveController.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.touchpad().and(driveController.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.touchpad().and(driveController.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driveController.L2().whileTrue(new RotationalAimCommand(drivetrain, () -> driveController.getLeftX() * -1, () -> driveController.getLeftY() * -1));
        //driveController.povDown().whileTrue(new IntakeRunCommand(m_intakeSubsystem));

                // Reset the field-centric heading on left bumper press.
        driveController.povLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
                
        drivetrain.registerTelemetry(logger::telemeterize);
        driveController.triangle().whileTrue(new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed)); // Not needed, button can be switched according to driver's preferences
        driveController.circle().whileTrue(new FeederCommand(m_feederSubsystem, FeederConstants.kReverseFeederMotorSpeed));
        // Intended to turn the feeder on after the fly-wheels have been on for a second
       driveController.R2().whileTrue(
    Commands.parallel(
        new ShooterCommand(m_shooterSubsystem, drivetrain),
        Commands.sequence(
            // Wait until the condition is met WITHOUT ending the whole group
            Commands.waitUntil(m_shooterSubsystem::shooterAtFireSpeed),
            new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed))));

        driveController.R1().whileTrue(new IntakeRunCommand(m_intakeSubsystem));
        driveController.povUp().whileTrue(new ClimbCommand(m_climbSubsystem, -0.5));
        driveController.povDown().whileTrue(new ClimbCommand(m_climbSubsystem, 0.5));
        driveController.povLeft().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING));
        driveController.povRight().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED));
        driveController.touchpad().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.HOME));
        driveController.PS().whileTrue(
            Commands.parallel(
                new IntakeRunCommand(m_intakeSubsystem)
                .beforeStarting(
                    Commands.sequence(
                        new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED),
                        Commands.waitUntil(() -> m_intakeSubsystem.pivotInPosition()),
                        new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING),
                        Commands.waitUntil(() -> m_intakeSubsystem.pivotInPosition())
                    )
                    .repeatedly()
                )
                .finallyDo(() -> {
                    m_intakeSubsystem.stopIntake();
                    m_intakeSubsystem.stopPivot();
                })
            )
        );
    }
        

    public void registerNamedCommands() {
        NamedCommands.registerCommand("Brake", drivetrain.applyRequest(() -> brake));
        NamedCommands.registerCommand("Intake", new IntakeRunCommand(m_intakeSubsystem));
        NamedCommands.registerCommand("Extend Climb", new ClimbCommand(m_climbSubsystem, -0.5));
        NamedCommands.registerCommand("Retract Climb", new ClimbCommand(m_climbSubsystem, 0.5));
        NamedCommands.registerCommand("Auto Aim at Outpost", new RotationalAimCommand(drivetrain, () -> driveController.getLeftX() * -1, () -> driveController.getLeftY() * -1));
        NamedCommands.registerCommand("Shoot", 
            Commands.parallel(
            new ShooterCommand(m_shooterSubsystem, drivetrain),
            Commands.sequence(
            // Wait until the condition is met WITHOUT ending the whole group
            Commands.waitUntil(m_shooterSubsystem::shooterAtFireSpeed),
            new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed))));
        NamedCommands.registerCommand("Pivot Indexing", new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING));
    }

            public Command getAutonomousCommand() {
                try{
        
                return autoChooser.getSelected();
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
            } 
        
        /*     // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }*/
}
}