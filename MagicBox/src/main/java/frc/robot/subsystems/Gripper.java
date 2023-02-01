// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static frc.robot.Constants.GripperConstants.*;

import java.util.Map;

import static frc.robot.Constants.ControlPanelConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.BeambreakConstants;
import frc.robot.PicoColorSensor.RawColor;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Gripper extends SubsystemBase {

  private final CANSparkMax m_gripperLeftMotor;
  private final CANSparkMax m_gripperRightMotor;

  // private final PicoColorSensor m_colorSensor;

  // public RawColor m_detectedColor;
  // public int m_proximity;
  public static String m_gripper_direction;

  private final NetworkTable m_gripperTable;
  private final NetworkTableEntry m_gripperStatus;

  // private final ShuffleboardTab m_controlPanelTab;
  // private final ShuffleboardLayout m_controlPanelStatus;
  // private final ShuffleboardLayout m_colorStatus;

  /** Creates a new Gripper. */
  public Gripper() {

// Initialize left and right motors.
m_gripperLeftMotor = new CANSparkMax(kGripperLeftMotor, MotorType.kBrushless);
m_gripperRightMotor = new CANSparkMax(kGripperRightMotor, MotorType.kBrushless);

m_gripperLeftMotor.restoreFactoryDefaults();
m_gripperRightMotor.restoreFactoryDefaults();

m_gripperLeftMotor.setIdleMode(IdleMode.kBrake);
m_gripperRightMotor.setIdleMode(IdleMode.kBrake);

// Sets color sensor.
// m_colorSensor = new PicoColorSensor();

// m_detectedColor = m_colorSensor.getRawColor0();
// m_proximity = m_colorSensor.getProximity0();

// Initializes network table for gripper and gripper status
m_gripperTable = NetworkTableInstance.getDefault().getTable("Gripper  ");
m_gripperStatus = m_gripperTable.getEntry("Gripper Running");

m_gripper_direction = "none";

// m_controlPanelTab = Shuffleboard.getTab(kShuffleboardTab);
//     m_controlPanelStatus = m_controlPanelTab.getLayout("Color Status", BuiltInLayouts.kList)
//       .withSize(3, 3)
//       .withProperties(Map.of("Label position", "TOP"));

//     m_colorStatus = m_controlPanelTab.getLayout("Color", BuiltInLayouts.kList)
//     .withSize(3, 3)
//     .withProperties(Map.of("Label position", "TOP", "Color when false", "black"));

    // shuffleboardInit();

  }

// private void shuffleboardInit() {
//     // Displays color detected as a color box
//     m_controlPanelStatus.addBoolean("Purple", () -> m_detectedColor.green > m_detectedColor.blue && m_proximity >= 80)
//       .withProperties(Map.of("Color when true", "Purple", "Color when false", "Black"));
//     m_controlPanelStatus.addBoolean("Yellow", () -> m_detectedColor.blue > m_detectedColor.green && m_detectedColor.blue - m_detectedColor.green >= 200 && m_proximity < 120 && m_proximity > 30) 
//       .withProperties(Map.of("Color when true", "Yellow", "Color when false", "Black"));

//     // Shows color values (RGB)
//     m_controlPanelStatus.addNumber("R", () -> m_detectedColor.red);
//     m_controlPanelStatus.addNumber("G", () -> m_detectedColor.green);
//     m_controlPanelStatus.addNumber("B", () -> m_detectedColor.blue);

//     // Proximity to ball
//     m_controlPanelStatus.addNumber("Ball Proximity", () -> m_proximity);

//   }

public void runGripper(double speed) {
m_gripperLeftMotor.set(speed);
m_gripperRightMotor.set(speed);
m_gripperStatus.setBoolean(true);
  }

public void stopGripper() {
  m_gripper_direction = "none";
  m_gripperLeftMotor.set(0);
  m_gripperRightMotor.set(0);
  m_gripperStatus.setBoolean(false);
}

public void intakeGripper() {
  m_gripper_direction = "intake";
  runGripper(kGripperIntakeMotorSpeed);
}

public void ejectGripper() {
  m_gripper_direction = "eject";
  runGripper(kGripperEjectMotorSpeed);
}


  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
// m_detectedColor = m_colorSensor.getRawColor0();
// m_proximity = m_colorSensor.getProximity0();
m_gripperStatus.setString(m_gripper_direction);
  }
}