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
  /** Creates a new Ultrasonic2024. */
  DigitalInput dsInput;
  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus; 
  // The ultrasonic sensor returns a value proportional to the distance
  // Here, the ratio of is 20.32mm to 1 volt

  public DiffuseSensor() {
    dsInput = new DigitalInput(0);

    m_controlPanelTab = Shuffleboard.getTab("Diffuse Sensor");
    m_controlPanelStatus = m_controlPanelTab.getLayout("Diffuse Sensor", BuiltInLayouts.kList)
    .withSize(3, 3);

    shuffleboardInit();
  }


  private void shuffleboardInit() {
    // Proximity to ball
    m_controlPanelStatus.addNumber("Distance", () -> getDistance());

  }

  public double getDistance() {
   // voltage = 2 * (dsInput.getAverageVoltage());
    //dist = scaleDtoV * voltage;
    
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
