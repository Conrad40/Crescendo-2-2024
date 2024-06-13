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
  //the motors are called krakens but they have a Talon motor controller built in and as programmers we only care about the motor controller.

  public Shooter() {
    //Setting the CAN IDs
    m_shooterMotorLeft = new TalonFX(CANIDConstants.kSHOOTER_LEFT_MOTOR_ID);
    m_shooterMotorRight = new TalonFX(CANIDConstants.kSHOOTER_RIGHT_MOTOR_ID);

    //set Inverted so that they go in different directions 
    m_shooterMotorLeft.setInverted(!ShooterConstants.kIS_INVERTED);// the "!" is the not opperator in java. Turn true to false and false to true.
    m_shooterMotorRight.setInverted(ShooterConstants.kIS_INVERTED);

    // set Neutral mode is the same as set Idle mode for the Sparks. the reason I set it to coast is to minimize damage to notes
    m_shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    m_shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //by default all subsystems have this method however you could delete it if you felt inclind to do so
    //most of the time periodic is unused execpt for putting stuff on the SmartDashboard
  }


  //defalts to full speed unless a value is given
  //In Java you can have mutiple methods with the same name IF they require differnet arguments
  public void Shoot(){
    //only ment for auto
    m_shooterMotorLeft.set(1);
    m_shooterMotorRight.set(1);
  }
  public void Shoot(double speed){
    m_shooterMotorLeft.set(speed);
    m_shooterMotorRight.set(speed);
  }
  public void Stop(){
    m_shooterMotorLeft.stopMotor();
    m_shooterMotorRight.stopMotor();
  }
}
