// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Vector2d;

public class DriveViaXboxController extends CommandBase {
  XboxController m_xboxController;
  SwerveDrive m_swerveDrive;
  /** Creates a new DriveViaXboxController. */
  public DriveViaXboxController(SwerveDrive swerveDrive, XboxController xboxController) {
    m_xboxController = xboxController;
    m_swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_xboxController.getPOV() == 0) {
      m_swerveDrive.resetFieldOrientation();
    }
    m_swerveDrive.driveFieldOriented(new Vector2d(-m_xboxController.getLeftX(), -m_xboxController.getLeftY()), -m_xboxController.getRightX());
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
