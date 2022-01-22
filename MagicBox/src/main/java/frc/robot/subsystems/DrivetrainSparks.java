// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class DrivetrainSparks extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Create Spark Maxes
  public final com.revrobotics.CANSparkMax leftSpark;
  public final com.revrobotics.CANSparkMax rightSpark;

  //Encoders
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

 // private final ADIS16470_IMU m_imu;

  private final DifferentialDrive m_drive;

  // Shuffleboards
  private final ShuffleboardTab m_drivetrainTab;
  private final ShuffleboardLayout m_drivetrainStatus;

  public boolean m_reverseDrive = false;

  private double leftSparkSpeed = -999.0;
  private double rightSparkSpeed = -999.0;
  
  public DrivetrainSparks() {
    // Spark Maxes
    leftSpark = new com.revrobotics.CANSparkMax(Constants.LEFT_SPARK_ID, MotorType.kBrushless);
    rightSpark = new com.revrobotics.CANSparkMax(Constants.RIGHT_SPARK_ID, MotorType.kBrushless);
    
    motorInit(leftSpark, Constants.kLeftReversedDefault);
    motorInit(rightSpark, Constants.kRightReversedDefault);

    leftSpark.setSmartCurrentLimit(Constants.STALL_LIMIT);
    rightSpark.setSmartCurrentLimit(Constants.STALL_LIMIT);

    leftSpark.setIdleMode(IdleMode.kBrake);
    rightSpark.setIdleMode(IdleMode.kBrake);

    // Encoders
    m_leftEncoder = leftSpark.getEncoder();
    m_rightEncoder = rightSpark.getEncoder();

    m_drive = new DifferentialDrive(leftSpark, rightSpark);
    
    // Initialize Shuffleboard
    m_drivetrainTab = Shuffleboard.getTab(Constants.kShuffleboardTabSpark);
    m_drivetrainStatus = m_drivetrainTab.getLayout("Status", BuiltInLayouts.kList)
      .withSize(3,3)
      .withProperties(Map.of("Label position", "TOP"));
    shuffleboardInit();
  }

  private void motorInit(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.kCurrentLimit);
    motor.setInverted(invert);

    encoderInit(motor.getEncoder());
  }

  private void encoderInit(RelativeEncoder encoder) {
    encoderReset(encoder);
  }

  private void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  private double getLeftDistance() {
    return -m_leftEncoder.getPosition();
  }

  private double getRightDistance() {
    return -m_rightEncoder.getPosition();
  }

  // private double getLeftSpeed() {
  //   return -m_leftEncoder.getVelocity();
  // }

  // private double getRightSpeed() {
  //   return -m_rightEncoder.getVelocity();
  // }

  //public double getRobotAngle() {
    //return m_imu.getAngle();
 // }

  private void shuffleboardInit() {
    m_drivetrainStatus.addNumber("Left Speed", () -> leftSparkSpeed);
    m_drivetrainStatus.addNumber("Right Speed", () -> rightSparkSpeed);
    m_drivetrainStatus.addNumber("Left Output", () -> leftSpark.get());
    m_drivetrainStatus.addNumber("Right Output", () -> rightSpark.get());
    //m_drivetrainStatus.addNumber("Left Position", () -> getLeftDistance());
    //m_drivetrainStatus.addNumber("Right Position", () -> getRightDistance());
    // m_drivetrainStatus.addNumber("Angle", () -> getRobotAngle());
     // m_drivetrainStatus.addBoolean("Reversed?", () -> m_reverseDrive);
  }
  
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    if (m_reverseDrive) {
      m_drive.tankDrive(rightPower/2, leftPower/2, squareInputs);
    }
    else {
      m_drive.tankDrive(leftPower/2, rightPower/2, squareInputs); 
    }
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftSparkSpeed = -m_leftEncoder.getVelocity();
    rightSparkSpeed = -m_rightEncoder.getVelocity();
  }
}
