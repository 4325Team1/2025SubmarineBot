// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightCommand extends Command {
  /** Creates a new LimeLightCommand. */
  public final LimeLightSubsystem m_LimelightCamera;
  public LimeLightCommand(LimeLightSubsystem subsystem) {
   m_LimelightCamera = subsystem;
   addRequirements(m_LimelightCamera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LimelightCamera.distanceFinder();
    m_LimelightCamera.limelight_range_proportional();
    m_LimelightCamera.limelight_range_proportional_ty();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
