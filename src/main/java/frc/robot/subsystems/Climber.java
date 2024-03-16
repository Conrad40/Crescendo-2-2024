// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ClimbConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax m_left;
  private CANSparkMax m_right;

  private SparkLimitSwitch m_leftForwardLimit;
  private SparkLimitSwitch m_leftReverseLimit;
  private SparkLimitSwitch m_rightForwardLimit;
  private SparkLimitSwitch m_rightReverseLimit;

  public Climber() {
    m_left = new CANSparkMax(CANIDConstants.kCLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_right = new CANSparkMax(CANIDConstants.kCLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    m_left.setIdleMode(IdleMode.kBrake);
    m_right.setIdleMode(IdleMode.kBrake);

    m_left.setInverted(!ClimbConstants.kIS_INVERTED);
    m_right.setInverted(ClimbConstants.kIS_INVERTED);

    m_leftForwardLimit = m_left.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_leftReverseLimit = m_left.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_rightForwardLimit = m_right.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_rightReverseLimit = m_right.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Climb(Double speed) {
    m_right.set(speed);
    m_left.set(speed);
  }



  public void Stop(){
m_right.stopMotor();
m_left.stopMotor();
  }

}
