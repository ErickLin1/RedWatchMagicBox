// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RunMotor;

public class OneMotor extends SubsystemBase {
  /** Creates a new OneMotor. */

  private final GenericEntry setMotorEncoder;
  private final ShuffleboardTab m_controlpanelTab;

  private final ShuffleboardLayout m_motorStatus;
  public final com.revrobotics.CANSparkMax m_motor;
  public final RelativeEncoder m_encoder;

  public OneMotor(int ID) {
    m_motor = new com.revrobotics.CANSparkMax(ID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(45);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);
    m_controlpanelTab = Shuffleboard.getTab("Control Panel");

    m_motorStatus = m_controlpanelTab.getLayout("Motor Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(0, 0)
      .withSize(2, 4);
      
    m_motorStatus.addNumber("Motor Speed", () -> getSpeed()); // Angle of shooter
    setMotorEncoder = m_motorStatus.add("Set Motor Speed", getSpeed()).getEntry();
    m_motorStatus.add(new RunMotor(this, setMotorEncoder.get().getDouble()));  


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
