package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class TestActuatorsCommand extends Command{
    private ShooterSubsystem m_shooterSubsystem;
    private final double setPosition;

    public TestActuatorsCommand(ShooterSubsystem shooterSubsystem, double position) {
        m_shooterSubsystem = shooterSubsystem;
        setPosition = position;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
       m_shooterSubsystem.testServo(setPosition);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
