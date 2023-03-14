// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDStrip;

import java.awt.Color;

public class LEDFlash extends CommandBase {
  /** Creates a new LEDFlash. */

  private Timer m_timer = new Timer();
  private LEDStrip m_ledStrip;
  private Color m_color1;
  private Color m_color2;
  private double m_time;
  final double m_DELAY = 0.25; // (seconds) How long to wait between each increment of the pattern



  public LEDFlash(LEDStrip ledStrip, Color color1, Color color2, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(ledStrip);
    m_color1 = color1;
    m_color2 = color2;
    m_ledStrip = ledStrip; 
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = m_timer.get();
    time = time - Math.floor(time/0.50) * 0.50;
    if (time < 0.22) {
      m_ledStrip.setAllToColor(m_color1.getRed(), m_color1.getGreen(), m_color1.getBlue());
    } else if (time < .25) {
      m_ledStrip.turnAllOff();
    } else if (time < 0.47) {
      m_ledStrip.setAllToColor(m_color2.getRed(), m_color2.getGreen(), m_color2.getBlue());
    } else {
      m_ledStrip.turnAllOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ledStrip.turnAllOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() > m_time;
  }
}
