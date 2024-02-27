// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libraries.AutonomousCommandSelector;
import frc.robot.Libraries.AutonomousCommands;
import frc.robot.Libraries.AutonomousSteps;
import frc.robot.Libraries.ConsoleAuto;

//I know it is not ideal but my plan is to make the most simplfied version of the older junk I made last year.
//Comp is in 4 days so functional is more important then best practice.
public class Autonomous extends SubsystemBase {
public enum Paths {
  BASIC(0, 0, 2.5, 0);

  private final double m_dStartX;
  private final double m_DStartY;
  private final double m_DMidX;
  private final double m_dMidY;
  private final double m_dEndX;
  private final double m_dEndY;

  private Paths(double dStartX, double dStartY, double dEndX, double dEndY) {
          this.m_dStartX = dStartX;
          this.m_DStartY = dStartY;
          this.m_DMidX = (dStartX + dEndX) / 2;
          this.m_dMidY = (dStartY + dEndY) / 2;
          this.m_dEndX = dEndX;
          this.m_dEndY = dEndY;
  }
}

  /** Creates a new Autonomous. */
  private DriveSubsystem m_drive;

  AutonomousCommandSelector<AutonomousSteps> m_autoCommand;
  private String kAUTO_TAB = "Autonomous";
  private String kSTATUS_PEND = "PEND";
  private String kSTATUS_ACTIVE = "ACTV";
  private String kSTATUS_DONE = "DONE";
  private String kSTATUS_SKIP = "SKIP";
  private String kSTATUS_NULL = "NULL";

  private int kSTEPS = 5;
  private boolean kRESET_ODOMETRY = true;

    ConsoleAuto m_ConsoleAuto;
  AutonomousCommands m_autoSelectCommand[] = AutonomousCommands.values();
  AutonomousCommands m_selectedCommand;

  public Autonomous() {

  }

  public void selectAutoCommand() {

    int autoSelectIx = m_ConsoleAuto.getROT_SW_0();
    m_iPatternSelect = autoSelectIx;
    if (autoSelectIx >= m_autoSelectCommand.length) {
      autoSelectIx = 0;
      m_iPatternSelect = 0;
    }
    // System.out.println(DriverStation.getAlliance().toString());
    boolean isAllianceRed = (DriverStation.getAlliance().toString() == "Red");
    m_allianceColor.setBoolean(isAllianceRed);

    m_selectedCommand = m_autoSelectCommand[autoSelectIx];
    m_strCommand = m_selectedCommand.toString();
    m_autoCmd.setString(m_strCommand);

    for (int ix = 0; ix < m_cmdSteps[autoSelectIx].length; ix++) {
      m_strStepList[ix] = m_cmdSteps[autoSelectIx][ix].getStrName();
      m_bStepSWList[ix] = m_cmdSteps[autoSelectIx][ix].isTrue();
      m_strStepStatusList[ix] = kSTATUS_PEND;
    }
    for (int ix = m_cmdSteps[autoSelectIx].length; ix < m_strStepList.length; ix++) {
      m_strStepList[ix] = "";
      m_bStepSWList[ix] = false;
      m_strStepStatusList[ix] = "";
    }

    for (int ix = 0; ix < kSTEPS; ix++) {
      m_step[ix].setString(m_strStepList[ix]);
      m_sw[ix].setValue(m_bStepSWList[ix]);
      m_st[ix].setString(m_strStepStatusList[ix]);
    }

    m_iWaitCount = m_ConsoleAuto.getROT_SW_1();
    m_iWaitLoop.setValue(m_iWaitCount);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
