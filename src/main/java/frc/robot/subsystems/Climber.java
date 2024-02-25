// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax m_LeftClimber;
  private CANSparkMax m_RightClimber;

  private SparkLimitSwitch m_leftFwdLimit;
  private SparkLimitSwitch m_leftRevLimit;

  private SparkLimitSwitch m_rightFwdLimit;
  private SparkLimitSwitch m_rightRevLimit;

  private final AHRS m_gyro = new AHRS();

  public Climber() {
    m_LeftClimber = new CANSparkMax(CANIDConstants.kLEFT_CLIMB_MOTOR_ID, MotorType.kBrushless);
    m_RightClimber = new CANSparkMax(CANIDConstants.kRIGHT_CLIMB_MOTOR_ID, MotorType.kBrushless);

    m_LeftClimber.setIdleMode(IdleMode.kBrake);
    m_RightClimber.setIdleMode(IdleMode.kBrake);

    m_leftFwdLimit = m_LeftClimber.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_leftRevLimit = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_rightFwdLimit = m_RightClimber.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_rightRevLimit = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    m_LeftClimber.setInverted(false);
    m_RightClimber.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Up(){
    //my math assumes that if the right side is higher the gyro is positve.
m_LeftClimber.set(.5+ m_gyro.getRoll()/90);
m_RightClimber.set(.5);
  }
  public void Down(){
    m_LeftClimber.set(-1);
    m_RightClimber.set(-1);
  }
  public void Left(){
    m_LeftClimber.set(1);
  }
  public void Right(){
    m_RightClimber.set(1);
  }
  public void Stop(){
    m_LeftClimber.stopMotor();
    m_RightClimber.stopMotor();
  }
}
