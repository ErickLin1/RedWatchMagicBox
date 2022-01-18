// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

//import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // Create Talons
  private final WPI_TalonSRX leftTalon;
  private final WPI_TalonSRX rightTalon;

  // Create Spark Maxes
  public final com.revrobotics.CANSparkMax leftMotor;
  //public final com.revrobotics.CANSparkMax leftMotor2;
  public final com.revrobotics.CANSparkMax rightMotor;
  //public final com.revrobotics.CANSparkMax rightMotor2;

  //Encoders
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

 // private final ADIS16470_IMU m_imu;

  private final DifferentialDrive m_drive;

  // Shuffleboards
  private final ShuffleboardTab m_drivetrainTab;
  private final ShuffleboardLayout m_drivetrainStatus;

  public boolean m_reverseDrive = false;

  public Drivetrain() {

    // Spark Maxes
    leftMotor = new com.revrobotics.CANSparkMax(Constants.LEFT_MOTOR_ID, MotorType.kBrushless);
    //leftMotor2 = new com.revrobotics.CANSparkMax(Constants.LEFT_MOTOR2_ID, MotorType.kBrushless);
    rightMotor = new com.revrobotics.CANSparkMax(Constants.RIGHT_MOTOR_ID, MotorType.kBrushless);
    //rightMotor2 = new com.revrobotics.CANSparkMax(Constants.RIGHT_MOTOR2_ID, MotorType.kBrushless);

    motorInit(leftMotor, Constants.kLeftReversedDefault);
    //motorInit(leftMotor2, Constants.kLeftReversedDefault);
    motorInit(rightMotor, Constants.kRightReversedDefault);
    //motorInit(rightMotor2, Constants.kRightReversedDefault);

    leftMotor.setSmartCurrentLimit(Constants.STALL_LIMIT);
    rightMotor.setSmartCurrentLimit(Constants.STALL_LIMIT);
   // leftMotor2.setSmartCurrentLimit(Constants.STALL_LIMIT);
    //rightMotor2.setSmartCurrentLimit(Constants.STALL_LIMIT);

    leftMotor.setIdleMode(IdleMode.kBrake);
    //leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
    //rightMotor2.setIdleMode(IdleMode.kBrake);

    //leftMotor2.follow(leftMotor);
    //rightMotor2.follow(rightMotor);

    m_leftEncoder = leftMotor.getEncoder();
    m_rightEncoder = rightMotor.getEncoder();

   // m_imu = new ADIS16470_IMU();
   // m_imu.reset();

    // Talons
    leftTalon = new WPI_TalonSRX(Constants.kLeftTalonPort);
    rightTalon = new WPI_TalonSRX(Constants.kRightTalonPort);
    
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();



    m_drive = new DifferentialDrive(leftMotor, rightMotor);

    m_drivetrainTab = Shuffleboard.getTab(Constants.kShuffleboardTab);
    m_drivetrainStatus = m_drivetrainTab.getLayout("Status", BuiltInLayouts.kList)
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

  private double getLeftSpeed() {
    return -m_leftEncoder.getVelocity();
  }

  private double getRightSpeed() {
    return -m_rightEncoder.getVelocity();
  }

  //public double getRobotAngle() {
    //return m_imu.getAngle();
 // }

  private void shuffleboardInit() {
    m_drivetrainStatus.addNumber("Left Speed", () -> getLeftSpeed());
    m_drivetrainStatus.addNumber("Right Speed", () -> getRightSpeed());
    m_drivetrainStatus.addNumber("Left Position", () -> getLeftDistance());
    m_drivetrainStatus.addNumber("Right Position", () -> getRightDistance());
   // m_drivetrainStatus.addNumber("Angle", () -> getRobotAngle());
    m_drivetrainStatus.addBoolean("Reversed?", () -> m_reverseDrive);
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
  }
}
