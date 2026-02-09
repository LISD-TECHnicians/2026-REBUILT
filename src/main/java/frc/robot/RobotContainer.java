// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder.IndexingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.IntakeConstants;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.Position;
import frc.robot.commands.IntakeOscillateCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.IntakeRunCommand;

public class RobotContainer {

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
   
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().whileTrue(new IntakeRunCommand(m_intakeSubsystem));
    m_driverController.b().whileTrue(new IntakePositionCommand(m_intakeSubsystem, Position.INDEXING));
    m_driverController.a().whileTrue(new IntakePositionCommand(m_intakeSubsystem, Position.DEPLOYED));
    m_driverController.y().whileTrue(new IntakePositionCommand(m_intakeSubsystem, Position.HOME));

    //m_driverController.
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
