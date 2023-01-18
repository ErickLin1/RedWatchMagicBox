// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MeasuringPotentiometer extends SubsystemBase {

  private final AnalogPotentiometer pot;
  private final ShuffleboardTab m_controlPanelTab;
  private final ShuffleboardLayout m_controlPanelStatus; 
  private double pot_val;

  /** Creates a new MeasuringPotentiometer. */
  public MeasuringPotentiometer() {

    pot = new AnalogPotentiometer(1);
    m_controlPanelTab = Shuffleboard.getTab("stringpot");
    m_controlPanelStatus = m_controlPanelTab.getLayout("String Pot", BuiltInLayouts.kList)
    .withSize(3, 3)
    .withProperties(Map.of("Label position", "TOP"));

    shuffleboardInit();

  }

  private void shuffleboardInit() {
    // Proximity to ball
    m_controlPanelStatus.addNumber("Pot Value", () -> pot_val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pot_val = (pot.get())*50;
  }
}
