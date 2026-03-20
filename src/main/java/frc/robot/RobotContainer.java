// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants.DriverConstants;
import frc.robot.Constants.RobotConstants.FeederConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.Constants.RobotConstants.ClimbConstants;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.RotationalAimCommand;
import frc.robot.commands.RunIndexerCommand;
import frc.robot.commands.ShooterCommand;
//import frc.robot.commands.TestActuatorsCommand;
import frc.robot.commands.UpdateVisionMeasurementCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Position;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.subsystems.ClimbSubsystem;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double slowSpeedCoef = .75;
    //private double slowSpeedCoefRotational = .60;
    
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
    private final CommandXboxController driveController = new CommandXboxController(DriverConstants.kDriveControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(DriverConstants.kOperaterControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    ;

    public final LimelightSubsystem limelight = new LimelightSubsystem("limelight-front");
    public final LimelightSubsystem limelight2 = new LimelightSubsystem("limelight-rear");
    private final Trigger updateVisionTrigger = new Trigger(() -> LimelightHelpers.getTV("limelight-front") && LimelightHelpers.getTA("limelight-front") > .07);
    private final Trigger updateVisionTrigger2 = new Trigger(() -> LimelightHelpers.getTV("limelight-rear"));
    private final UpdateVisionMeasurementCommand updateVisionOdometry = new UpdateVisionMeasurementCommand(drivetrain, limelight);
    private final UpdateVisionMeasurementCommand updateVisionOdometry2 = new UpdateVisionMeasurementCommand(drivetrain, limelight2);
    private final SendableChooser<Command> autoChooser;
    

    public RobotContainer() {
        configureBindings();

        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * 0.8) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed * 0.8) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate * 1.50) // Drive counterclockwise with negative X (left)
                    .withDeadband(DriverConstants.kDriveControllerDeadband)
                    )
        );

        updateVisionTrigger.whileTrue(updateVisionOdometry);
        //updateVisionTrigger2.whileTrue(updateVisionOdometry2);
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        driveController.a().toggleOnTrue(drivetrain.applyRequest(() -> brake));
            
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        driveController.rightBumper().whileTrue(drivetrain.applyRequest(() ->  
            drive.withVelocityX(-driveController.getLeftY() * MaxSpeed * 0.2)
            .withVelocityY(-driveController.getLeftX() * MaxSpeed * 0.2)
            .withRotationalRate(-driveController.getRightX() * MaxAngularRate * 1.5)
            ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driveController.leftTrigger().whileTrue(new RotationalAimCommand(drivetrain, () -> driveController.getLeftX() * -1, () -> driveController.getLeftY() * -1));

                // Reset the field-centric heading on left bumper press.
        driveController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
                
        drivetrain.registerTelemetry(logger::telemeterize);
        driveController.y().whileTrue(new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed)); // Not needed, button can be switched according to driver's preferences
        driveController.b().whileTrue(
            Commands.parallel(
            new FeederCommand(m_feederSubsystem, FeederConstants.kReverseFeederMotorSpeed),
            new RunIndexerCommand(m_shooterSubsystem, -1.0)));
        // Intended to turn the feeder on after the fly-wheels have been on for a second
        driveController.rightTrigger().whileTrue(
    Commands.parallel(
            new ShooterCommand(m_shooterSubsystem, drivetrain),
            Commands.sequence(
            // Wait until the condition is met WITHOUT ending the whole group
            Commands.waitSeconds(1.75),//Until(m_shooterSubsystem::isIndividualMotorAtSpeed),
            Commands.parallel(
                new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed * 0.666),
                new IndexerCommand(m_shooterSubsystem)))));

        driveController.rightBumper().whileTrue(new IntakeRunCommand(m_intakeSubsystem, -IntakeConstants.kIntakeSpeedRunCoef));
        operatorController.b().whileTrue(new IntakeRunCommand(m_intakeSubsystem, IntakeConstants.kIntakeSpeedRunCoef));
        operatorController.povUp().whileTrue(new ClimbCommand(m_climbSubsystem, ClimbConstants.kExtendClimbSpeed));
        operatorController.povDown().whileTrue(new ClimbCommand(m_climbSubsystem, ClimbConstants.kRetractClimbSpeed));
        driveController.povRight().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING));
        driveController.povLeft().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED));
        driveController.start().onTrue(new IntakePositionCommand(m_intakeSubsystem, Position.HOME));
        driveController.x().whileTrue(
        Commands.sequence(
            new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED),
            Commands.waitUntil(() -> m_intakeSubsystem.pivotInPosition()),
            Commands.waitSeconds(0.1), // testing a wait to prevent instant change in pivot position state.

            new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING),
            Commands.waitUntil(() -> m_intakeSubsystem.pivotInPosition()),
            Commands.waitSeconds(0.1)  
        )
        .repeatedly()
        .finallyDo(() -> {
            m_intakeSubsystem.stopIntake();
            m_intakeSubsystem.stopPivot();
        })
    );
            
    }
        

    public void registerNamedCommands() {
        //NamedCommands.registerCommand("Brake", drivetrain.applyRequest(() -> brake));
        NamedCommands.registerCommand("Intake", new IntakeRunCommand(m_intakeSubsystem, -IntakeConstants.kIntakeSpeedRunCoef));
        //NamedCommands.registerCommand("Extend Climb", new ClimbCommand(m_climbSubsystem, ClimbConstants.kExtendClimbSpeed));
        //NamedCommands.registerCommand("Retract Climb", new ClimbCommand(m_climbSubsystem, ClimbConstants.kRetractClimbSpeed));
        NamedCommands.registerCommand("Auto Aim at Hub", new AutoAimCommand(drivetrain));
        NamedCommands.registerCommand("Shoot", 
            Commands.parallel(
            new ShooterCommand(m_shooterSubsystem, drivetrain),
            Commands.sequence(
            // Wait until the condition is met WITHOUT ending the whole group
            Commands.waitSeconds(1.0),//Until(m_shooterSubsystem::isIndividualMotorAtSpeed),
            Commands.parallel(
                new FeederCommand(m_feederSubsystem, FeederConstants.kFeederMotorSpeed * 0.666),
                new IndexerCommand(m_shooterSubsystem)))));
    
        //NamedCommands.registerCommand("Pivot Feeding", new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING));
        //NamedCommands.registerCommand("Pivot Home", new IntakePositionCommand(m_intakeSubsystem, Position.HOME));
        NamedCommands.registerCommand("Pivot Deployed", new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED));
        /*NamedCommands.registerCommand("Pivot Oscillate", 
            Commands.parallel( //Heavily Experimental, be ready on E-Stop when testing
                new IntakeRunCommand(m_intakeSubsystem, -IntakeConstants.kIntakeSpeedRunCoef) 
                .beforeStarting(  // verify the correction for intake motor running.
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
            ));
    }*/}

            public Command getAutonomousCommand() {
                try{
        
                return autoChooser.getSelected();
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
        }
    }
}