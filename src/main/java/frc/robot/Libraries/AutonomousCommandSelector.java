// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Libraries;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

/** Add your docs here. */
public class AutonomousCommandSelector<K> {

    private final Map<K, Command> m_autoCommand = new HashMap<>();
    private K m_defaultChoice = null;

public AutonomousCommandSelector() {}

    public void setDefaultOption(K name, Command object) {
        requireNonNullParam(name, "name", "setDefaultOption");

        m_defaultChoice = name;
        addOption(name, object);
    }

    public void addOption(K name, Command command) {
        m_autoCommand.put(name, command);
    }

    public Command getSelected(K autoCmdName) {
        Command autoCommand = m_autoCommand.get(autoCmdName);
        boolean bIsCommandFound = autoCommand != null;
        SmartDashboard.putBoolean("Auto Found", bIsCommandFound);
        if (!bIsCommandFound) {
            autoCommand = m_autoCommand.get(m_defaultChoice);
        }
        return autoCommand;
    }


}