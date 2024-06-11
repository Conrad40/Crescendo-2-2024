// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCount extends Command {
  /** Creates a new WaitForCount. */

  //the old code had this, I grabed it because I mistakenly thought it would be useful for the shooting command. I am keeping it because it is also used in auto.
  //I did simplfy it from the old code.
  // It is better to just use Commands.waitSeconds()
  private Timer m_timer = new Timer();
  double m_duration;
  public WaitForCount(double duration) {
    m_duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }
}
