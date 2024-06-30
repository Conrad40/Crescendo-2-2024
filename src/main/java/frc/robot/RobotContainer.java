// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.AutoControl;
import frc.robot.Commands.AutoSelect;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.DeployIntake;
import frc.robot.Commands.NoteIntake;
import frc.robot.Commands.RetractIntake;
import frc.robot.Commands.ShootNote;
import frc.robot.Commands.WaitForCount;
import frc.robot.Libraries.ConsoleAuto;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final Shooter m_Shooter = new Shooter();
        private final Intake m_Intake = new Intake();
        private final Climber m_Climber = new Climber();
       
        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        //the weird box to the right of the laptop on the DS
        private final ConsoleAuto m_consoleAuto = new ConsoleAuto(OIConstants.kAUTONOMOUS_CONSOLE_PORT);

        private final Autonomous m_autonomous = new Autonomous(m_consoleAuto, m_robotDrive,m_Shooter,m_Intake);// due to time contraits I gave up on doing auto the fancy way these three lines and the 

        // autonomous files simply are no longer used in code. However I did not delete them due to the amout of time it would take. You can now use them as examples of what to do.
        private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);
        private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_robotDrive,m_Shooter,m_Intake);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();
                // m_Shooter.setDefaultCommand(Commands.run(
                // () ->
                // m_Shooter.Shoot(MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                // .5) * .5 + .5),
                // m_Shooter));
                // Configure default commands

                m_Climber.setDefaultCommand(new RunCommand(() -> m_Climber.Stop(), m_Climber));
                //New programmers: please note that when using a RunCommand you have to give it a subsystem otherwise it wont work. But it will not tell you that you forgot it.
                // the reason it doesnt require a subsystem is because the method you give it might not use a subsystem, but if it does you have to give it control of that subsystem.
                //this could change next year.
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                // Multiply by max speed to map the joystick unitless
                                                                // inputs to actual units.
                                                                // This will map the [-1, 1] to [max speed backwards,
                                                                // max speed forwards],
                                                                // converting them to actual units.
                                                                //"MathUtil.applyDeadband" is used because when the conroller joysticks are released
                                                                //they can still read a small non-zero number when they should be reading zero
                                                                -MathUtil.applyDeadband(
                                                                                -m_driverController.getLeftY(), .08),
                                                                MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(), .08),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(), .08),

                                                                false, false, !m_Intake.isOut()),
                                                m_robotDrive));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                //this is were you should be configuring the buttons 

                m_driverController.povUp().whileTrue(Commands.run(() -> m_Climber.Climb(1.0)));
                m_driverController.povDown().whileTrue(Commands.run(() -> m_Climber.Climb(-1.0)));
                m_driverController.povLeft().whileTrue(Commands.run(() -> m_Climber.ArmIndependient(-1.0)));
                m_driverController.povRight().whileTrue(Commands.run(() -> m_Climber.ArmIndependient(1.0)));

                
                m_driverController
                                .leftBumper()
                                .whileTrue(new DeployIntake(m_Intake).unless(() -> m_Intake.isNoteIn())
                                                .andThen(new NoteIntake(m_Intake)
                                                                .unless(() -> m_Intake.isNoteIn()))
                                                .andThen(new RetractIntake(m_Intake)));
                // should deploy then run intake until a note is in it and then stops pulling
                // note and retracts intake.

                m_driverController
                                .a()
                                .whileTrue(new RetractIntake(m_Intake));
                // make command to deploy intake and spit note out of intake

                m_driverController
                                .leftTrigger(.75)
                                .whileTrue(new RetractIntake(m_Intake).andThen(new ShootNote(m_Shooter, m_Intake)));
                m_driverController.y().onFalse(Commands.runOnce(() -> m_Intake.holdNote())
                                .andThen(Commands.runOnce(() -> m_Shooter.Stop())));
                // Retracts Intake, starts shooter motors, waits N seconds (currently 1), has
                // intake spit note into shooter, then waits .25 seconds to let note leave.
                m_driverController
                                .y()
                                .whileTrue(Commands.parallel(Commands.run(() -> m_Intake.intakeNote()),
                                                Commands.run(() -> m_Shooter.Shoot(-.3))));
                m_driverController
                                .b()
                                .whileTrue(Commands.run(() -> m_Intake.dropNote()));
                m_driverController
                                .b()
                                .onFalse(Commands.run(() -> m_Intake.holdNote()));

                m_driverController
                                .x()
                                .whileTrue(Commands.run(() -> m_robotDrive.setX()));

        }
        public Command getAutoSelect() {
               return m_autoSelect;
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                return m_autoCommand;
                /* 
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setReversed(true)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(Math.PI)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, new Rotation2d(0))),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(1.5, 0, new Rotation2d(Math.PI)),
                                config);

                TrajectoryConfig config2 = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setReversed(false)
                                .setKinematics(DriveConstants.kDriveKinematics);

                // Add kinematics to ensure max speed is actually obeyed

                Trajectory BackTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, new Rotation2d(0))),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(1.5, 0, new Rotation2d(0)),
                                config2);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                SwerveControllerCommand swerveControllerCommandBack = new SwerveControllerCommand(
                                BackTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);
                // Reset odometry to the initial pose of the trajectory, run path following
                // command, then stop at the end.


                //a switch command that gets the position of the Rotery switch(0) on the auto side of the console.
                // YOU SHOULD ALWAYS HAVE A OPTIONAL WAIT for auto if the path involes any action. Helps pevent auto collistions with other teams.
                //0 = nothing
                //1 = waits a few seconds based off of Rotery switch 1 auto side of the console and then shoots a note.
                //2 = Optional wait, followed by a drive out
                //3 = Optional wait, shoots a note, drive out.
                //4 = Optional wait, shoots a note, drive out while intaking a note.
                //5 = Optional wait, shoots a note, drive out while intaking a note, drives back, shoots 2nd note
                //6+ = the Rotery switch we used only has 6 positions but if it ever reads 6 or higher the default is do nothing.
                switch (m_consoleAuto.getROT_SW_0()) {
                        case 0:
                                return null;

                        case 1:
                                return Commands.sequence(Commands.waitSeconds(m_consoleAuto.getROT_SW_1()),
                                                new ShootNote(m_Shooter, m_Intake));
                        case 2:
                                return Commands.sequence(Commands.waitSeconds(m_consoleAuto.getROT_SW_1()),
                                                new InstantCommand(
                                                                () -> m_robotDrive.resetOdometry(
                                                                                exampleTrajectory.getInitialPose())),
                                                swerveControllerCommand);
                        case 3:
                                return Commands.sequence(Commands.waitSeconds(m_consoleAuto.getROT_SW_1()),
                                                new ShootNote(m_Shooter, m_Intake),
                                                new InstantCommand(
                                                                () -> m_robotDrive.resetOdometry(
                                                                                exampleTrajectory.getInitialPose())),
                                                swerveControllerCommand);
                        case 4:
                                return Commands.sequence(Commands.waitSeconds(m_consoleAuto.getROT_SW_1()),
                                                new ShootNote(m_Shooter, m_Intake), Commands.parallel(
                                                                Commands.sequence(new DeployIntake(m_Intake)
                                                                                .unless(() -> m_Intake.isNoteIn())
                                                                                .andThen(new NoteIntake(m_Intake)
                                                                                                .unless(() -> m_Intake
                                                                                                                .isNoteIn()))
                                                                                .andThen(new RetractIntake(m_Intake))),
                                                                Commands.sequence(
                                                                                new InstantCommand(
                                                                                                () -> m_robotDrive
                                                                                                                .resetOdometry(
                                                                                                                                exampleTrajectory
                                                                                                                                                .getInitialPose())),
                                                                                swerveControllerCommand)));
                        case 5:
                                return Commands.sequence(Commands.waitSeconds(m_consoleAuto.getROT_SW_1()),
                                                new ShootNote(m_Shooter, m_Intake),
                                                Commands.parallel(
                                                                Commands.sequence(new DeployIntake(m_Intake)
                                                                                .unless(() -> m_Intake.isNoteIn())
                                                                                .andThen(new NoteIntake(m_Intake)
                                                                                                .unless(() -> m_Intake
                                                                                                                .isNoteIn()))
                                                                                .andThen(new RetractIntake(m_Intake))),
                                                                Commands.sequence(
                                                                                new InstantCommand(
                                                                                                () -> m_robotDrive
                                                                                                                .resetOdometry(
                                                                                                                                exampleTrajectory
                                                                                                                                                .getInitialPose())),
                                                                                swerveControllerCommand)),
                                                Commands.sequence(
                                                                new InstantCommand(
                                                                                () -> m_robotDrive.resetOdometry(
                                                                                                BackTrajectory
                                                                                                                .getInitialPose())),
                                                                swerveControllerCommandBack),
                                                new ShootNote(m_Shooter, m_Intake));
                        default:
                                return null;
                                //without the default being a return null, the code would crash if the dial was in a position that was not coded for. 
                                //All switch statments should include a default behavor. Note java normaly doesn't require a default but it is best practice.

                }*/

                /*
                 * if (m_consoleAuto.getButton(2)) {
                 * return new ShootNote(m_Shooter, m_Intake);
                 * }
                 * return Commands.sequence(
                 * new InstantCommand(
                 * () -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                 * swerveControllerCommand);
                 * // new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false, false)));
                 */
        }
}
