// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.differentialDriveSparks;
import frc.robot.commands.toggleSolenoid;

/** Creates a control panel in Shuffleboard that displays all important information and controls. Contains all shuffleboard related code. */
public class ControlPanel extends SubsystemBase {
  /** Creates a new ControlPanel. */

  private final ShuffleboardTab m_ShuffleboardTab;
  private final ShuffleboardLayout m_ShuffleboardLayout;
  private final ShuffleboardLayout m_SparkStatus;
  private final ShuffleboardLayout m_TalonStatus;

  private final NetworkTableEntry LeftMotorSpeed;
  private final NetworkTableEntry RightMotorSpeed;

  public ControlPanel(Climber m_climber, DrivetrainSparks m_drivetrainSparks, DrivetrainTalons m_drivetrainTalons) {
    // Initialize Control Panel Shuffleboard
    m_ShuffleboardTab = Shuffleboard.getTab(Constants.kShuffleboardTab);

    // Set up layouts
    m_ShuffleboardLayout = m_ShuffleboardTab.getLayout("Motor Controls", BuiltInLayouts.kList)
    .withPosition(3, 0)
    .withSize(3, 3);
    m_SparkStatus = m_ShuffleboardTab.getLayout("Spark Status", BuiltInLayouts.kList)
    .withSize(2,2)
    .withPosition(6,0)
    .withProperties(Map.of("Label position", "TOP"));
    m_TalonStatus = m_ShuffleboardTab.getLayout("Talon Status", BuiltInLayouts.kList)
    .withSize(2,3)
    .withPosition(6,2)
    .withProperties(Map.of("Label position", "TOP"));

    // Set up motor controls
    LeftMotorSpeed = m_ShuffleboardLayout.add("Left Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    RightMotorSpeed = m_ShuffleboardLayout.add("Right Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    // Turns on the motors and reads the shuffleboard's motor speed values
    m_ShuffleboardLayout.add("Run Motors", new differentialDriveSparks(() -> LeftMotorSpeed.getDouble(0), () -> RightMotorSpeed.getDouble(0), m_drivetrainSparks));

    // Set up Spark Status
    m_SparkStatus.addNumber("Left Speed", () -> m_drivetrainSparks.leftSparkSpeed);
    m_SparkStatus.addNumber("Right Speed", () -> m_drivetrainSparks.rightSparkSpeed);

    // Set up Talon Status
    m_TalonStatus.addNumber("Left Speed", () -> m_drivetrainTalons.getLeftSpeed());
    m_TalonStatus.addNumber("Right Speed", () -> m_drivetrainTalons.getRightSpeed());
    m_TalonStatus.addNumber("Left Output", () -> m_drivetrainTalons.getLeftOutput());
    m_TalonStatus.addNumber("Right Output", () -> m_drivetrainTalons.getRightOutput());

    // Enables or disables the solenoid
    m_ShuffleboardTab.add("Toggle Solenoid", new toggleSolenoid(m_climber))
      .withPosition(3, 3)
      .withSize(3, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
