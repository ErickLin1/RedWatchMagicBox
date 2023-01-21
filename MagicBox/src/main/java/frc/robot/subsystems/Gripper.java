// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PicoColorSensor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static frc.robot.Constants.GripperConstants.*;

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

  private final PicoColorSensor  m_colorSensor;

  public RawColor m_detectedColor;
  public int m_proximity;

  private final NetworkTable m_gripperTable;
  private final NetworkTableEntry m_gripperStatus;

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
m_colorSensor = new PicoColorSensor();

m_detectedColor = m_colorSensor.getRawColor0();
m_proximity = m_colorSensor.getProximity0();

// Initializes network table for gripper and gripper status
m_gripperTable = NetworkTableInstance.getDefault().getTable("Gripper  ");
m_gripperStatus = m_gripperTable.getEntry("Gripper Running");


  }

public void runGripper(double speed) {

m_gripperLeftMotor.set(speed);
m_gripperRightMotor.set(speed);
m_gripperStatus.setBoolean(true);
  }

public void stopGripper() {
  m_gripperLeftMotor.set(0);
  m_gripperRightMotor.set(0);
  m_gripperStatus.setBoolean(false);
}

public void intakeGripper() {
  runGripper(kGripperIntakeMotorSpeed);
}

public void ejectGripper() {
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
m_detectedColor = m_colorSensor.getRawColor0();
m_proximity = m_colorSensor.getProximity0();
  }
}
