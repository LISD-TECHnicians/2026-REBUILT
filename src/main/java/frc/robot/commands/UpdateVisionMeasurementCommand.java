package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/* Add correct robot swerve import(s) here */
import frc.robot.subsystems.LimelightSubsystem;
public class UpdateVisionMeasurementCommand extends Command {
    
    private final LimelightSubsystem m_limelightSubsystem
        = new LimelightSubsystem(""); // replace with odometry limelight name. 

    // private final SwerveSubsystem m_swerveSubsystem
        // = new SwerveSubsystem();

    private Pose2d m_robotPose2d; 
    private Optional<LimelightSubsystem.Measurement> m_measurement;

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
            /*  m_robotPose2d = m_swerveSubsystem.getState().Pose;
               m_measure = m_limelightSubsystem.getMeasurement(m_robotPose2d);
            if (m_measure.isPresent()) {
                m_swerveSubsystem.addVisionMeasurement(
                    m_measure.poseEstimate.pose, 
                    m_measure.poseEstimate.timestampSeconds,
                    m_measure.standardDeviations
                )
            }
            */ 
            }

        
    
    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
