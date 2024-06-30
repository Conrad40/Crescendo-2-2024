package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class AutoControl extends Command {
// Runs Auto based off of what it gets from the Autonomous subsystem
    private Command m_currentCommand;
    private Autonomous m_autonomous;
    private DriveSubsystem m_drive;
    private Shooter m_Shooter;
    private Intake m_intake;

    public AutoControl(Autonomous autonomous, DriveSubsystem drive,Shooter shooter,Intake intake) {
        m_autonomous = autonomous;
        m_drive = drive;
        m_intake = intake;
        m_Shooter = shooter;
        addRequirements(m_autonomous, m_drive, shooter, m_intake);
    }

    @Override
    public void initialize() {
        // getInstance();
        m_autonomous.initGetCommand();
        m_currentCommand = m_autonomous.getNextCommand();
        m_currentCommand.initialize();
    }

    @Override
    public void execute() {
        m_currentCommand.execute();
        // System.out.println("execute");
    }

    @Override
    public void end(boolean interrupted) {
        m_currentCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        boolean areWeThereYet = true;
        Command nextCommand = null;
        if (m_currentCommand.isFinished() == false) {
            areWeThereYet = false;
        } else {
            nextCommand = m_autonomous.getNextCommand();
            if (nextCommand != null) {
                switchCommand(nextCommand);
                areWeThereYet = false;
            }
        }
        return areWeThereYet;
    }

    // stops current command then goes to next one
    private void switchCommand(final Command cmd) {
        m_currentCommand.end(false);
        m_currentCommand = cmd;
        m_currentCommand.initialize();
    }

}