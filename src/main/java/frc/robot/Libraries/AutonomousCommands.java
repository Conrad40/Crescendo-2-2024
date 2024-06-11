// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Libraries;

/** Add your docs here. */
public enum AutonomousCommands {
    WAIT_SHOOT,
    WAIT_DRIVE,
    SHOOT_DRIVE,
    SHOOT_INTAKE,
    SHOOT_SHOOT;



    public String getSelectName() {
        return this.toString();
    }

    public int getSelectIx() {
        return this.ordinal();
    }
}