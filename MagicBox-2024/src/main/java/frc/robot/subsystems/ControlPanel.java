// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

public class ControlPanel extends SubsystemBase {

  private final ShuffleboardTab m_controlpanelTab;

  private final OneMotor m_onemotor;


  private final GenericEntry setMotorEncoder;

  private final ShuffleboardLayout m_motorStatus;
  /** Creates a new ControlPanel. */
  public ControlPanel(OneMotor onemotor) {
    m_onemotor = onemotor;

    m_controlpanelTab = Shuffleboard.getTab("Control Panel");

    m_motorStatus = m_controlpanelTab.getLayout("Motor Status", BuiltInLayouts.kList)
      .withProperties(Map.of("Label position", "TOP"))
      .withPosition(0, 0)
      .withSize(2, 4);
      
    m_motorStatus.addNumber("Motor Speed", () -> m_onemotor.getSpeed()); // Angle of shooter
    setMotorEncoder = m_motorStatus.add("Set Motor Speed", m_onemotor.getSpeed()).getEntry();
    m_motorStatus.add(runOnce(()->{onemotor.setSpeed(setMotorEncoder.get().getDouble());}));   
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
