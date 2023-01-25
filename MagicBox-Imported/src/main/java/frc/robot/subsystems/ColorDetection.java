// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.PicoColorSensor.RawColor;

import java.util.Map;

import static frc.robot.Constants.ControlPanelConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class ColorDetection extends SubsystemBase {
  private final PicoColorSensor m_colorSensor;

  public RawColor m_detectedColor;
  public int proximity;

  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus;
  private final ShuffleboardLayout m_colorStatus;

  /** Creates a new Color. */
  public ColorDetection() {
    // Creates new color sensor and shufffleboard
    m_colorSensor = new PicoColorSensor();
    m_controlPanelTab = Shuffleboard.getTab(kShuffleboardTab);
    m_controlPanelStatus = m_controlPanelTab.getLayout("Color Status", BuiltInLayouts.kList)
      .withSize(3, 3)
      .withProperties(Map.of("Label position", "TOP"));

    m_colorStatus = m_controlPanelTab.getLayout("Color", BuiltInLayouts.kList)
    .withSize(3, 3)
    .withProperties(Map.of("Label position", "TOP", "colorWhenFalse", "black"));

    shuffleboardInit();
  }

  private void shuffleboardInit() {
    // Displays color detected as a color box
    m_controlPanelStatus.addBoolean("Red", () -> m_detectedColor.red > m_detectedColor.blue && m_detectedColor.red >= 0.7)
      .withProperties(Map.of("Color when true", "Red", "Color when false", "Black"));
    m_controlPanelStatus.addBoolean("Blue", () -> m_detectedColor.blue > m_detectedColor.red && m_detectedColor.blue >= 0.7) 
      .withProperties(Map.of("Color when true", "Cyan", "Color when false", "Black"));
    // m_controlPanelStatus.addBoolean("Green", () -> m_detectedColor.green >= 0.5);

    // Shows color values (RGB)
    m_controlPanelStatus.addNumber("R", () -> m_detectedColor.red);
    m_controlPanelStatus.addNumber("G", () -> m_detectedColor.green);
    m_controlPanelStatus.addNumber("B", () -> m_detectedColor.blue);

    // Proximity to ball
    m_controlPanelStatus.addNumber("Ball Proximity", () -> proximity);

    m_colorStatus.withProperties(Map.of("colorWhenTrue", m_detectedColor)); // FIX COLOR NOT BEING SHOWN. Need to get color value

  }

  @Override
  public void periodic() {
    // Collects color data and proximity on repeat
    m_detectedColor = m_colorSensor.getRawColor0();
    proximity = m_colorSensor.getProximity0();
  }
}