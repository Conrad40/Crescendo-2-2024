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
        // The driver's controller

        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        private final ConsoleAuto m_consoleAuto = new ConsoleAuto(OIConstants.kAUTONOMOUS_CONSOLE_PORT);
        private final Autonomous m_autonomous = new Autonomous(m_consoleAuto, m_robotDrive);
        
        private final AutoSelect m_autoSelect = new AutoSelect(m_autonomous);
        private final AutoControl m_autoCommand = new AutoControl(m_autonomous, m_robotDrive);

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
                                                                .5 * -MathUtil.applyDeadband(
                                                                                -m_driverController.getLeftY(), .08),
                                                                .5 * MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX(), .08),
                                                                .5 * -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX(), .08),

                                                                false, false),
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

                m_driverController
                                .leftBumper()
                                .whileTrue(new DeployIntake(m_Intake)
                                                .andThen(new NoteIntake(m_Intake)
                                                                .unless(() -> m_Intake.isNoteIn()))
                                                .andThen(new RetractIntake(m_Intake)));
                // should deploy then run intake until a note is in it and then stops pulling
                // note and retracts intake.
                // but might retract intake anyway if the bumper is no longer being held.

                m_driverController
                                .a()
                                .whileTrue(new RetractIntake(m_Intake));
                // make command to deploy intake and spit note out of intake

                m_driverController
                                .leftTrigger(.75)
                                .whileTrue(new RetractIntake(m_Intake).andThen(new ShootNote(m_Shooter, m_Intake)));
                // Retracts Intake, starts shooter motors, waits N seconds (currently 1), has
                // intake spit note into shooter, then waits .25 seconds to let note leave.

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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutoSelect() {
                return m_autoSelect;
        }

        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                // Command autoCommand = ;
                return m_autoCommand;
        }
}
