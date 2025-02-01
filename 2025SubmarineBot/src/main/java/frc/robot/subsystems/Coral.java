// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.commands.LimelightCameraCMD;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  public DriveSubsystem m_driveSubsystem;
  public LimeLightSubsystem m_cameraData;
  public double yTargetDistance = 0;
  public boolean alignment;
  public double wantedDistance = 0;
  public CoralSubsystem(DriveSubsystem driveSubsystem, LimeLightSubsystem camera)
   {
    m_driveSubsystem = driveSubsystem;
    m_cameraData = camera;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void levelOneRight()
  {
    //SmartDashboard.putString("pipeline 0", "false");
    // 1. check if right triger is being presses
    if(RobotContainer.m_driverController.getRightTriggerAxis() > 0.5)
    // 2. if so switch the pipline of the limelight (profiles) to number 0 to only read the right side april tag
    {m_cameraData.SwitchPipeline(0);
    SmartDashboard.putString("pipeline 0", "ture");
    }
    if(m_cameraData.haveTarget) 
    return;
      
      alignment = m_cameraData.distanceFinder - wantedDistance;
      if(!alignment)
      {
        m_driveSubsystem.VisionDrive(-(m_cameraData.goalHeightInches - m_cameraData.distanceFinder()), 0 ,0); 
        return;
      }
      else 
      { 
        m_driveSubsystem.stopModules();
      }
    }
      public void levelOneLeft()
      {
        SmartDashboard.putString("pipeline 1", "false");
        // 1. check if right triger is being presses
        if(RobotContainer.m_driverController.getLeftTriggerAxis() > 0.5)
        // 2. if so switch the pipline of the limelight (profiles) to number 1 to only read the left side april tag
        {m_cameraData.SwitchPipeline(1);
        SmartDashboard.putString("pipeline 1", "true");
        }
        // 3. line robot up with the april tag
    
      
      }
  
}
