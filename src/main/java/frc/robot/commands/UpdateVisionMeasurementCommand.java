package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.Utils;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


/* Add correct robot swerve import(s) here */
import frc.robot.subsystems.LimelightSubsystem;
public class UpdateVisionMeasurementCommand extends Command {
    

    CommandSwerveDrivetrain m_commandSwerveDrivetrain;
    LimelightSubsystem m_limelightSubsystem;


    public UpdateVisionMeasurementCommand (CommandSwerveDrivetrain subsystem, LimelightSubsystem limelightSubsystem){
        m_commandSwerveDrivetrain = subsystem;
        m_limelightSubsystem = limelightSubsystem;
    }

    private Pose2d m_robotPose2d; 
    // private Optional<LimelightSubsystem.Measurement> m_measurement = LimelightSubsystem.Measurement();

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
            m_robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue("limelight-front");
            
            if (LimelightHelpers.getTV("limelight-front")) {
                m_commandSwerveDrivetrain.addVisionMeasurement(
                    m_robotPose2d, m_limelightSubsystem.getAdjustedTimestamp());
            } 
            
            }


        
    
    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
