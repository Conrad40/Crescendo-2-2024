// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX m_shooterMotorLeft;
  private TalonFX m_shooterMotorRight;

  public Shooter() {
    m_shooterMotorLeft = new TalonFX(CANIDConstants.kSHOOTER_LEFT_MOTOR_ID);
    m_shooterMotorRight = new TalonFX(CANIDConstants.kSHOOTER_RIGHT_MOTOR_ID);

    m_shooterMotorLeft.setInverted(!ShooterConstants.kIS_INVERTED);
    m_shooterMotorRight.setInverted(ShooterConstants.kIS_INVERTED);

    m_shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    m_shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  //defalts to full speed unless a value is given
  public void Shoot(){
    //only ment for auto
    m_shooterMotorLeft.set(1);
    m_shooterMotorRight.set(1);
  }
  public void Shoot(double speed){
    m_shooterMotorLeft.set(speed);
    m_shooterMotorRight.set(speed);
  }
}
