// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
private RelativeEncoder m_Encoder;
private RelativeEncoder m_REncoder;
 final AHRS m_gyro = new AHRS();
 private double m_flatRoll;
  public Climber() {
m_flatRoll = m_gyro.getRoll();
      System.out.println(m_flatRoll);
    

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
m_Encoder = m_left.getEncoder();
m_REncoder = m_right.getEncoder();
m_REncoder.setPosition(0);
m_Encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("roll", m_gyro.getRoll());
    SmartDashboard.putNumber("climb encoder", m_Encoder.getPosition());
    if (m_leftForwardLimit.isPressed()){
      m_Encoder.setPosition(0);
     
    }
    if (m_rightForwardLimit.isPressed()){
       m_REncoder.setPosition(0);
    }
    // This method will be called once per scheduler run
  }

  public void Climb(Double speed) {
    m_right.set(speed* (((m_gyro.getRoll())/20)+.75));
    m_left.set(speed* (((-m_gyro.getRoll())/20)+.75));
  }



  public void Stop(){
m_right.stopMotor();
m_left.stopMotor();
  }

}
