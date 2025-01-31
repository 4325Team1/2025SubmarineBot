// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  public double distanceFromLimelightToGoalInches;
  public double targetingForwardSpeed;
  public double targetingForwardSpeedTy;

  public LimeLightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double distanceFinder(){
        

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 90 - 63; // 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 40.75; //

    // distance from the target to the floor
    double goalHeightInches = 72.0; //

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); 

    //calculate distance
    distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  public double limelight_range_proportional_ty()
  { 

    // With Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableValue ty = table.getValue("ty");  
    double kP = .1;
    targetingForwardSpeedTy = ty.getDouble()* kP;

   // targetingForwardSpeedTy *= DriveConstants.kMaxSpeed;
    targetingForwardSpeedTy *= -1.0;
    return targetingForwardSpeedTy;

  }

  public double limelight_range_proportional()
  {
    // Without Table : work
    double currentDistance = distanceFinder();
    double kP = .1;
    double desireDistance = 60;
    double distanceError = desireDistance - currentDistance;

    targetingForwardSpeed = distanceError * kP;
    //targetingForwardSpeed *= DriveConstants.kMaxSpeed;
    targetingForwardSpeed *= -1.0;

    return targetingForwardSpeed;
  }
  public void SwitchPipeline(int pipelineNumber)
  {
   NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipelineNumber);
  }
}
