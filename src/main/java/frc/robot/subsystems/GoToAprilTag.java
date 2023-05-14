// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoToAprilTag extends CommandBase {
  int m_apriltagIdOfInterest;
  ApriltagInfo m_apriltagInfo;
  SwerveDrive m_swerveDrive;
  double m_tolerance = 0.03; // how many meters from desired position is good enough
  double m_targetDistance  = 1.0; // how many meters to stand off from apriltag
  /** Creates a new GoToAprilTag. */
  public GoToAprilTag(SwerveDrive swerveDrive, ApriltagInfo apriltagInfo, int apriltagIdOfInterest, double targetDistance, double tolerance) {
    m_apriltagIdOfInterest = apriltagIdOfInterest;
    m_swerveDrive = swerveDrive;
    m_targetDistance = targetDistance;
    m_tolerance = tolerance;
    m_apriltagInfo = apriltagInfo;
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ApriltagInfo.ApriltagRecord apriltagRecord = m_apriltagInfo.getApriltagRecord(m_apriltagIdOfInterest);
    // TODO: ApriltagRecord should have method to compute camera position relative to tag
    return false;
  }
}
