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
    m_flatRoll = m_gyro.getRoll(); // What is left over from an attemt to make the arms auto balance the robot
                                   // based off of the gyro.
    // System.out.println(m_flatRoll);

    m_left = new CANSparkMax(CANIDConstants.kCLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
    m_right = new CANSparkMax(CANIDConstants.kCLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

    m_left.setIdleMode(IdleMode.kBrake);
    m_right.setIdleMode(IdleMode.kBrake);
    // Brake mode is important for this subsystem other wise the robot would not be
    // able to hold itself up after teleop is over.
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
    // since the arms have a "soft" limit going up it has to know where they are and
    // the only known refrance is when they are all the way down.
    // so I had It reset the encoders to 0 when they are at the bottom
    if (m_leftForwardLimit.isPressed()) {
      m_Encoder.setPosition(0);

    }
    if (m_rightForwardLimit.isPressed()) {
      m_REncoder.setPosition(0);
    }

  }

  public void Climb(Double speed) {
    //changes relitive speed of arms based off of the gyro
    m_right.set(speed * (((m_gyro.getRoll() - 4.6) / 10) + 1));
    m_left.set(speed * (((-m_gyro.getRoll() + 4.6) / 10) + 1));
  }

  public void ArmIndependient(Double speed) {
    //allows for manualy changing the hight of the arms sepretly
    m_left.set(speed);
    m_right.set(-speed);
  }

  public void Stop() {
    m_right.stopMotor();
    m_left.stopMotor();
  }

}
