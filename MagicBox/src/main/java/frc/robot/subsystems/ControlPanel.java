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
import static frc.robot.Constants.DriveSpark.*;
import static frc.robot.Constants.ControlPanelConstants.*;
import frc.robot.commands.differentialDriveSparks;
import frc.robot.commands.differentialDriveTalons;
import frc.robot.commands.toggleSolenoid;

/** Creates a control panel in Shuffleboard that displays all important information and controls. Contains all shuffleboard related code. */
public class ControlPanel extends SubsystemBase {
  /** Creates a new ControlPanel. */
  private final Beambreak m_beambreak;

  private final ShuffleboardTab m_ShuffleboardTab;
  private final ShuffleboardLayout m_SparkControls;
  private final ShuffleboardLayout m_TalonControls;
  private final ShuffleboardLayout m_SparkStatus;
  private final ShuffleboardLayout m_TalonStatus;

  private final NetworkTableEntry LeftSparkMotor;
  private final NetworkTableEntry RightSparkMotor;
  private final NetworkTableEntry LeftSparkRPM;
  private final NetworkTableEntry RightSparkRPM;
  private final NetworkTableEntry LeftTalonMotor;
  private final NetworkTableEntry RightTalonMotor;

  public ControlPanel(Climber m_climber, DrivetrainSparks m_drivetrainSparks, DrivetrainTalons m_drivetrainTalons) {
    // Initialize Control Panel Shuffleboard
    m_ShuffleboardTab = Shuffleboard.getTab(kShuffleboardTab);

    // Set up layouts
    m_SparkControls = m_ShuffleboardTab.getLayout("Spark Motor Controls", BuiltInLayouts.kList)
    .withPosition(3, 0)
    .withSize(2, 3);
    m_TalonControls = m_ShuffleboardTab.getLayout("Talon Motor Controls", BuiltInLayouts.kList)
    .withPosition(7, 0)
    .withSize(2, 3);
    m_SparkStatus = m_ShuffleboardTab.getLayout("Spark Status", BuiltInLayouts.kList)
    .withSize(2,2)
    .withPosition(5,0)
    .withProperties(Map.of("Label position", "TOP"));
    m_TalonStatus = m_ShuffleboardTab.getLayout("Talon Status", BuiltInLayouts.kList)
    .withSize(2,2)
    .withPosition(5,2)
    .withProperties(Map.of("Label position", "TOP"));

    // Set up spark motor controls
    LeftSparkMotor = m_SparkControls.add("Left Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    RightSparkMotor = m_SparkControls.add("Right Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    // Turns on the motors and reads the shuffleboard's motor speed values
    m_SparkControls.add("Run Motors", new differentialDriveSparks(() -> LeftSparkMotor.getDouble(0), () -> RightSparkMotor.getDouble(0), m_drivetrainSparks));

    // Set up Spark RPM controls
    LeftSparkRPM = m_ShuffleboardTab.add("Left RPM", maxSparkRPM)
      .withPosition(0, 3)
      .getEntry();
    RightSparkRPM = m_ShuffleboardTab.add("Right RPM", maxSparkRPM)
      .withPosition(1, 3)
      .getEntry();
    m_ShuffleboardTab.add("Run Motors RPM", new differentialDriveSparks(() -> getSpeedFromRPM(LeftSparkRPM.getDouble(0)), () -> getSpeedFromRPM(RightSparkRPM.getDouble(0)), m_drivetrainSparks)).withPosition(2, 3);

    // Set up talon motor controls
    LeftTalonMotor = m_TalonControls.add("Left Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    RightTalonMotor = m_TalonControls.add("Right Motor Speed", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
    // Turns on the motors and reads the shuffleboard's motor speed values
    m_TalonControls.add("Run Motors", new differentialDriveTalons(() -> LeftTalonMotor.getDouble(0), () -> RightTalonMotor.getDouble(0), m_drivetrainTalons));

    // Set up Spark Status
    m_SparkStatus.addNumber("Left Speed", () -> m_drivetrainSparks.leftSparkSpeed);
    m_SparkStatus.addNumber("Right Speed", () -> m_drivetrainSparks.rightSparkSpeed);

    // Set up Talon Status
    // m_TalonStatus.addNumber("Left Speed", () -> m_drivetrainTalons.getLeftSpeed()); // not working
    // m_TalonStatus.addNumber("Right Speed", () -> m_drivetrainTalons.getRightSpeed()); // not working
    // m_TalonStatus.addNumber("Left Output", () -> m_drivetrainTalons.getLeftOutput());
    // m_TalonStatus.addNumber("Right Output", () -> m_drivetrainTalons.getRightOutput());

    // Enables or disables the solenoid
    m_ShuffleboardTab.add("Toggle Solenoid", new toggleSolenoid(m_climber))
      .withPosition(3, 3)
      .withSize(2, 1);
    
    // Set up beam break status
    m_beambreak = new Beambreak();
    m_ShuffleboardTab.addBoolean("Beam Status", () -> m_beambreak.get())
    .withPosition(7, 3)
    .withSize(2, 1);
  }

  /**
   * Converts RPM to Speed so the robot can run at that RPM.
   * @param RPM the rotations per minute to convert
   * @return the converted speed to get that rpm
   */
  private double getSpeedFromRPM(double RPM) {
    int maxRPM = maxSparkRPM;
    double speed = RPM / maxRPM;

    if (speed > 1.0) {
      return 1.0;
    }

    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
