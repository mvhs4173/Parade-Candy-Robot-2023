// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDStrip;

public class FlashLEDLaunchPattern extends CommandBase {
  private final int TARGET_FLASHES = 3;
  private final double ON_TIME = .15;
  private LEDStrip ledStrip;

  private int flashCount;
  private Timer timer = new Timer();
  private Timer onTimer = new Timer();

  /** Creates a new FlashLEDLaunchPattern. */
  public FlashLEDLaunchPattern(LEDStrip ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledStrip = ledStrip;
    timer.reset();
    timer.start();
    flashCount = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    onTimer.reset();
    ledStrip.turnAllOff();
    flashCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= Constants.launchDelay / TARGET_FLASHES) {
      ledStrip.setAllToColor(6, 126, 152);
      flashCount++;
      timer.reset();
      onTimer.reset();
      onTimer.start();
    }
    if (onTimer.get() >= ON_TIME) {
      ledStrip.turnAllOff();
      onTimer.reset();
      onTimer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledStrip.turnAllOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flashCount >= TARGET_FLASHES;
  }
}
