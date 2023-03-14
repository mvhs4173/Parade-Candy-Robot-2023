// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.CandyCannon;
// import frc.robot.subsystems.LEDStrip;

public class LaunchCandy extends CommandBase {

  CandyCannon cannon;
  CommandBase cmdFlashLEDs;
  CommandBase cmdRunPattern;

  boolean hasLaunched;
  Timer timer = new Timer();
  
  /** Creates a new LaunchCandy. */
  public LaunchCandy(CandyCannon cannon, CommandBase flashLEDCommand, CommandBase runPatternCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cannon = cannon;
    cmdFlashLEDs = flashLEDCommand;
    cmdRunPattern = runPatternCommand;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmdRunPattern.cancel();
    cmdFlashLEDs.schedule();
    timer.reset();
    timer.start();
    hasLaunched = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasLaunched) {
      if (cannon.getIsExtended()) {
        cannon.setIsExtended(false);
      }
      if (timer.get() >= Constants.launchDelay) {
        cannon.setIsExtended(true);
        hasLaunched = true;
        timer.reset();
        cmdFlashLEDs.cancel();
      }
    } else {
      // RobotContainer.ledStrip.chaseTrail(10, 0, 255, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cannon.setIsExtended(false);
    cmdRunPattern.schedule();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasLaunched && timer.get() >= Constants.launchDelay;
  }
}
