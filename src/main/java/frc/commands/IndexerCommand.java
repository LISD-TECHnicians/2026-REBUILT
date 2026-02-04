package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command{
    private IndexerSubsystem m_indexerSubsystem;

    public IndexerCommand (IndexerSubsystem indexerSubsystem) {
        m_indexerSubsystem = indexerSubsystem;
        addRequirements(m_indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_indexerSubsystem.setIndexerMotorSpeed(IndexerConstants.kIndexerMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}

