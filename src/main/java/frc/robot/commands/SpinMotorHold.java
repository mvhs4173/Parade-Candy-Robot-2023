// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoMotor;

public class SpinMotorHold extends CommandBase {
  private NeoMotor m_NeoMotor;
  private double m_voltage;
  /** Creates a new SpinMotorHold. */
  public SpinMotorHold(NeoMotor neoMotor, double voltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_NeoMotor = neoMotor;
    m_voltage = voltage;
    addRequirements(m_NeoMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_NeoMotor.setVoltage(m_voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_NeoMotor.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
