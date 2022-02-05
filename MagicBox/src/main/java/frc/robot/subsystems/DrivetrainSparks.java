// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

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

  // public double leftSparkSpeed;
  // public double rightSparkSpeed;
  
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
    // m_drivetrainTab = Shuffleboard.getTab(Constants.kShuffleboardTab);
    // m_drivetrainStatus = m_drivetrainTab.getLayout("Spark Status", BuiltInLayouts.kList)
    //   .withSize(2,2)
    //   .withPosition(6,0)
    //   .withProperties(Map.of("Label position", "TOP"));
    // shuffleboardInit();
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

  // private double getLeftDistance() {
  //   return -m_leftEncoder.getPosition();
  // }

  // private double getRightDistance() {
  //   return -m_rightEncoder.getPosition();
  // }

  public double getLeftSpeed() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return m_rightEncoder.getVelocity();
  }

  //public double getRobotAngle() {
    //return m_imu.getAngle();
 // }

  // private void shuffleboardInit() {
  //   m_drivetrainStatus.addNumber("Left Speed", () -> leftSparkSpeed);
  //   m_drivetrainStatus.addNumber("Right Speed", () -> rightSparkSpeed);
  //   // m_drivetrainStatus.addNumber("Left Output", () -> leftSpark.get());
  //   // m_drivetrainStatus.addNumber("Right Output", () -> rightSpark.get());
  //   //m_drivetrainStatus.addNumber("Left Position", () -> getLeftDistance());
  //   //m_drivetrainStatus.addNumber("Right Position", () -> getRightDistance());
  //   // m_drivetrainStatus.addNumber("Angle", () -> getRobotAngle());
  // }
  
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    m_drive.tankDrive(leftPower, rightPower, squareInputs);
  }

  public void stopDrive() {
    m_drive.tankDrive(0, 0);
  }

  public void setLeftSpeed(double speed) {
    leftSpark.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightSpark.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // leftSparkSpeed = m_leftEncoder.getVelocity();
    // rightSparkSpeed = m_rightEncoder.getVelocity();
  }
}
