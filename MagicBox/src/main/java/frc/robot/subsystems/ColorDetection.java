// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import java.util.Map;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.I2C;

public class ColorDetection extends SubsystemBase {
  private final ColorSensorV3 m_colorSensor;

  private Color m_detectedColor;
  private int proximity;

  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus;

  /** Creates a new Color. */
  public ColorDetection() {
    // Creates new color sensor and shufffleboard
    m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_controlPanelTab = Shuffleboard.getTab(Constants.kShuffleboardTab);
    m_controlPanelStatus = m_controlPanelTab.getLayout("Status", BuiltInLayouts.kList)
      .withSize(3, 4)
      .withProperties(Map.of("Label position", "TOP"));

    shuffleboardInit();
  }

  private void shuffleboardInit() {
    // Displays color detected as a color box
    m_controlPanelStatus.addBoolean("Red", () -> m_detectedColor.red > m_detectedColor.blue && m_detectedColor.red >= 0.3);
    m_controlPanelStatus.addBoolean("Blue", () -> m_detectedColor.blue > m_detectedColor.red && m_detectedColor.blue >= 0.3);
    m_controlPanelStatus.addBoolean("Green", () -> m_detectedColor.green >= 0.5);

    // Shows color values (RGB)
    m_controlPanelStatus.addNumber("R", () -> m_detectedColor.red);
    m_controlPanelStatus.addNumber("G", () -> m_detectedColor.green);
    m_controlPanelStatus.addNumber("B", () -> m_detectedColor.blue);

    // Proximity to ball
    m_controlPanelStatus.addNumber("Ball Proximity", () -> proximity);
  }

  @Override
  public void periodic() {
    // Collects color data and proximity on repeat
    m_detectedColor = m_colorSensor.getColor();
    proximity = m_colorSensor.getProximity();
  }
}
