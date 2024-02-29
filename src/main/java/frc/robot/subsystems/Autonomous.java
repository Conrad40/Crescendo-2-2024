// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.SwerveDriveController;
import frc.robot.Commands.WaitForCount;
import frc.robot.Libraries.AutonomousCommandSelector;
import frc.robot.Libraries.AutonomousCommands;
import frc.robot.Libraries.AutonomousSteps;
import frc.robot.Libraries.ConsoleAuto;
import frc.robot.Libraries.StepState;

//I know it is not ideal but my plan is to make the most simplfied version of the older junk I made last year.
//Comp is in 4 days so functional is more important then best practice.
public class Autonomous extends SubsystemBase {
  enum Paths {
    BASIC(0, 0, 2.5, 0),
    BEND(0, 0, 1, .5, 2, 0);

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

    private Paths(double dStartX, double dStartY, double dMidX, double dMidY, double dEndX, double dEndY) {
      this.m_dStartX = dStartX;
      this.m_DStartY = dStartY;
      this.m_DMidX = dMidX;
      this.m_dMidY = dMidY;
      this.m_dEndX = dEndX;
      this.m_dEndY = dEndY;
    }

    double getStartX() {
      return m_dStartX;
    }

    double getStartY() {
      return m_DStartY;
    }

    double getMidX() {
      return m_DMidX;
    }

    double getMidY() {
      return m_dMidY;
    }

    double getEndX() {
      return m_dEndX;
    }

    double getEndY() {
      return m_dEndY;
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

  private String m_strCommand = " ";
  private String[] m_strStepList = { "", "", "", "", "" };
  private boolean[] m_bStepSWList = { false, false, false, false, false };
  private String[] m_strStepStatusList = { "", "", "", "", "" };

  private ShuffleboardTab m_tab = Shuffleboard.getTab(kAUTO_TAB);

  // private final SimpleWidget m_autoCmd = m_tab.add("Selected Pattern", "");
  private GenericEntry m_autoCmd = m_tab.add("Selected Pattern", "")
      .withPosition(0, 0)
      .withSize(2, 1)
      .getEntry();

  private GenericEntry m_iWaitLoop = m_tab.add("WaitLoop", 0)
      .withWidget(BuiltInWidgets.kDial)
      .withPosition(0, 1)
      .withSize(2, 2)
      .withProperties(Map.of("min", 0, "max", 5))
      .getEntry();

  private GenericEntry m_allianceColor = m_tab.add("Alliance", true)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .withProperties(Map.of("colorWhenTrue", "Red", "colorWhenFalse", "Blue"))
      .withPosition(0, 3)
      .withSize(1, 1)
      .getEntry();

  private GenericEntry m_step[] = { m_tab.add("Step0", m_strStepList[0])
      .withWidget(BuiltInWidgets.kTextView)
      .withPosition(2, 0)
      .withSize(1, 1)
      .getEntry(),
      m_tab.add("Step1", m_strStepList[1])
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(3, 0)
          .withSize(1, 1)
          .getEntry(),
      m_tab.add("Step2", m_strStepList[2])
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(4, 0)
          .withSize(1, 1)
          .getEntry(),
      m_tab.add("Step3", m_strStepList[3])
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(5, 0)
          .withSize(1, 1)
          .getEntry(),
      m_tab.add("Step4", m_strStepList[4])
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(6, 0)
          .withSize(1, 1)
          .getEntry()
  };

  public Autonomous(GenericEntry[] m_step) {
    this.m_step = m_step;
  }

  private GenericEntry m_sw[] = { m_tab.add("Step0Sw", m_bStepSWList[0])
      .withPosition(2, 1)
      .withSize(1, 1)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry(),
      m_tab.add("Step1Sw", m_bStepSWList[1])
          .withPosition(3, 1)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry(),
      m_tab.add("Step2Sw", m_bStepSWList[2])
          .withPosition(4, 1)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry(),
      m_tab.add("Step3Sw", m_bStepSWList[3])
          .withPosition(5, 1)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry(),
      m_tab.add("Step4Sw", m_bStepSWList[4])
          .withPosition(6, 1)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .getEntry()
  };
  private GenericEntry m_st[] = { m_tab.add("Stat0", m_strStepStatusList[0])
      .withPosition(2, 2)
      .withSize(1, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry(),
      m_tab.add("Stat1", m_strStepStatusList[1])
          .withPosition(3, 2)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry(),
      m_tab.add("Stat2", m_strStepStatusList[2])
          .withPosition(4, 2)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry(),
      m_tab.add("Stat3", m_strStepStatusList[3])
          .withPosition(5, 2)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry(),
      m_tab.add("Stat4", m_strStepStatusList[4])
          .withPosition(6, 2)
          .withSize(1, 1)
          .withWidget(BuiltInWidgets.kTextView)
          .getEntry()
  };
  private int m_iPatternSelect;

  private Command m_currentCommand;
  private boolean m_bIsCommandDone = false;
  private int m_stepIndex;
  private int m_iWaitCount;
  private Trajectory m_drive3Trajectory;
  private WaitCommand m_wait1;
  private StepState m_stepWait1Sw1;
  private WaitCommand m_wait2;
  private StepState m_stepWait2Sw1;
  private StepState m_stepWait2Sw2;
  private StepState m_stepWait2SwAB;
  private WaitForCount m_waitForCount;
  private StepState m_stepWaitForCount;
  private StepState m_stepDrive3Path;

  private SwerveDriveController m_drivePath;
  private StepState m_stepDrivePath;
  private SwerveDriveController m_testReadFilePath;
  private StepState m_stepTestReadFile;

  private Command m_turnPath;
  private StepState m_stepturnPath;
  //private String m_path1JSON = "output/Blue center.wpilib.json";
 // private String m_pathb2JSON = "output/blue drive out.wpilib.json";
  //private String m_pathB1JSON = "output/Red"
  private Trajectory m_trajPath1;

  private AutonomousSteps m_currentStepName;
  private StepState[][] m_cmdSteps;

  public Autonomous(ConsoleAuto consoleAuto, DriveSubsystem drive) {
    m_ConsoleAuto = consoleAuto;
    m_drive = drive;

    m_selectedCommand = m_autoSelectCommand[0];
    m_strCommand = m_selectedCommand.toString();
    m_autoCommand = new AutonomousCommandSelector<AutonomousSteps>();
    m_iPatternSelect = 0;

    m_stepWait1Sw1 = new StepState(AutonomousSteps.WAIT1, m_ConsoleAuto.getSwitchSupplier(1));

    m_stepWait2Sw1 = new StepState(AutonomousSteps.WAIT2, m_ConsoleAuto.getSwitchSupplier(1));
    m_stepWait2Sw2 = new StepState(AutonomousSteps.WAIT2, m_ConsoleAuto.getSwitchSupplier(2));

    m_stepWaitForCount = new StepState(AutonomousSteps.WAITLOOP);

    var thetaController = new ProfiledPIDController(2, 0, 0, new Constraints(5, 1));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivePath = new SwerveDriveController(genTrajectory(Paths.BASIC),
        kRESET_ODOMETRY, m_drive, thetaController);
    m_autoCommand.addOption(AutonomousSteps.DRIVE, m_drivePath);
    m_stepDrivePath = new StepState(AutonomousSteps.DRIVE,
        m_ConsoleAuto.getSwitchSupplier(3));

    m_testReadFilePath = new SwerveDriveController(genTrajectory(Paths.BASIC), kRESET_ODOMETRY, drive, thetaController);
    m_autoCommand.addOption(AutonomousSteps.TEST, m_testReadFilePath);
    m_stepTestReadFile = new StepState(AutonomousSteps.TEST);

    m_cmdSteps = new StepState[][] {
        { m_stepWaitForCount, m_stepDrivePath }
        // { m_stepWaitForCount, m_stepTestReadFile }
        // { m_stepWaitForCount, m_stepturnPath }
        // { m_stepWaitForCount, m_stepMoveArm, m_stepPlaceConeM, m_stepDrive3Path }
    };
  }

  public Trajectory genTrajectory(Paths path) {
    // System.out.println(path.getEndX());
    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(path.getStartX(), path.getStartY(), new Rotation2d(0)),
        List.of(new Translation2d(path.getMidX(), path.getMidY())),
        new Pose2d(path.getEndX(), path.getEndY(), new Rotation2d(0)),
        m_drive.getTrajConfig());
  }

  // read externally generated trajectory (path) from an external file in the
  // standard "deploy" path
  // these are generated from a standard tool such as pathweaver
  private Trajectory readPaths(String jsonPath) {
    Trajectory trajectory = null;
    try {
      Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(jsonPath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
    } catch (IOException ex) {
      System.out.println("Reading the path failed");
    }
    return trajectory;
  }

  public void selectAutoCommand() {

    int autoSelectIx = m_ConsoleAuto.getROT_SW_0();
    m_iPatternSelect = autoSelectIx;
    if (autoSelectIx >= m_autoSelectCommand.length) {
      autoSelectIx = 0;
      m_iPatternSelect = 0;
    }

   // boolean isAllianceRed = (DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
    //m_allianceColor.setBoolean(isAllianceRed);
 m_allianceColor.setBoolean(true);
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

  public void initGetCommand() {
    m_stepIndex = -1;

  }

  public Command getWaitCommand(double seconds) {
    return Commands.waitSeconds(seconds);
  }

  public Command getNextCommand() {
    m_currentStepName = null;
    m_currentCommand = null;
    String completionAction = kSTATUS_DONE;

    while (m_currentCommand == null && !m_bIsCommandDone) {
      m_currentStepName = getNextActiveCommand(completionAction);
      if (m_currentStepName != null) {
        switch (m_currentStepName) {
          case WAIT1:
            m_currentCommand = getWaitCommand(1);
            break;
          case WAIT2:
            m_currentCommand = getWaitCommand(2);
            break;
          case WAITLOOP:
            m_currentCommand = getWaitCommand(m_ConsoleAuto.getROT_SW_1());
            break;
          default:
            m_currentCommand = m_autoCommand.getSelected(m_currentStepName);
            break;
        }

        if (m_currentCommand == null) {
          completionAction = kSTATUS_NULL;
        }
      }
    }
    return m_currentCommand;
  }

  private AutonomousSteps getNextActiveCommand(String completionAction) {

    // System.out.println("getNextActiveCommand");

    AutonomousSteps stepName = null;

    while (stepName == null && !m_bIsCommandDone) {
      if (m_stepIndex >= 0 && m_stepIndex < kSTEPS) {
        m_strStepStatusList[m_stepIndex] = completionAction;
        m_st[m_stepIndex].setString(m_strStepStatusList[m_stepIndex]);
      }
      m_stepIndex++;
      if (m_stepIndex >= m_cmdSteps[m_iPatternSelect].length) {
        m_bIsCommandDone = true;
      } else {
        if (m_stepIndex < kSTEPS) {
          m_bStepSWList[m_stepIndex] = m_cmdSteps[m_iPatternSelect][m_stepIndex].isTrue();
          m_sw[m_stepIndex].setValue(m_bStepSWList[m_stepIndex]);
          // System.out.println("Step Boolean" + m_bStepSWList [m_stepIndex]);
        }
        if (m_cmdSteps[m_iPatternSelect][m_stepIndex].isTrue()) {
          m_strStepStatusList[m_stepIndex] = kSTATUS_ACTIVE;
          m_st[m_stepIndex].setString(m_strStepStatusList[m_stepIndex]);
          stepName = m_cmdSteps[m_iPatternSelect][m_stepIndex].getName();
        } else {
          completionAction = kSTATUS_SKIP;
        }
      }
    }

    return stepName;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isCommandDone() {
    return m_bIsCommandDone;
  }
}
