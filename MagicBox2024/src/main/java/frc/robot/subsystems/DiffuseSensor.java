// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DiffuseSensor extends SubsystemBase {
  /** Creates a new Diffuse Sensor. */
  DigitalInput dsInput;
  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus; 

  public DiffuseSensor() {
    dsInput = new DigitalInput(99);

    m_controlPanelTab = Shuffleboard.getTab("Diffuse Sensor");
    m_controlPanelStatus = m_controlPanelTab.getLayout("Diffuse Sensor", BuiltInLayouts.kList)
    .withSize(3, 3);

    shuffleboardInit();
  }


  private void shuffleboardInit() {
    // Proximity to ball
    m_controlPanelStatus.addNumber("Distance", () -> getDistance());

  }

  public boolean objectDetected() {
    return !dsInput.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Object present:", objectPresent());
  }
}
