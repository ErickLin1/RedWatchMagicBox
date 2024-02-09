// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OneMotor extends SubsystemBase {
  /** Creates a new OneMotor. */

  public final com.revrobotics.CANSparkMax m_motor;
  public final RelativeEncoder m_encoder;

  public OneMotor(int ID) {
    m_motor = new com.revrobotics.CANSparkMax(ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(45);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);



  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public void stopMotor() {
    m_motor.set(0);
  }

  public double getSpeed() {
    return m_encoder.getVelocity();
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public void resetPosition() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
