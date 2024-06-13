// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDriveController extends SwerveControllerCommand {
  /** Creates a new SwerveDriveController. */

  private DriveSubsystem m_drive;
  private Trajectory m_trajectory;
  private boolean m_resetOdometry;

  // dont do this, It did work but it is NOT WORTH the extra hassle to get it to work. 
  public SwerveDriveController(Trajectory trajectory, boolean resetOdometry, DriveSubsystem drive, ProfiledPIDController thetaController) {

    super(trajectory, drive::getPose, DriveConstants.kDriveKinematics, 
     new PIDController(1, 0, 0), new PIDController(1, 0, 0), thetaController,
        drive::setModuleStates, drive);
    // Use addRequirements() here to declare subsystem dependencies.
addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_resetOdometry) {
      m_drive.resetOdometry(m_trajectory.getInitialPose());
    }
  }
  /*
   * // Called every time the scheduler runs while the command is scheduled.
   * 
   * @Override
   * public void execute() {
   * }
   * 
   * // Called once the command ends or is interrupted.
   * 
   * @Override
   * public void end(boolean interrupted) {
   * }
   * 
   * // Returns true when the command should end.
   * 
   * @Override
   * public boolean isFinished() {
   * return false;
   * }
   */
}