package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */ 
  public final com.revrobotics.CANSparkMax bottomMotor;
  public final RelativeEncoder m_bottomEncoder;
  private final DigitalInput m_ballDetect;
  
  public Indexer() {
    //init motor
    bottomMotor = new com.revrobotics.CANSparkMax(Constants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    motorInit(bottomMotor, Constants.kTopReversedDefault);
    bottomMotor.setSmartCurrentLimit(Constants.STALL_LIMIT);
    bottomMotor.setIdleMode(IdleMode.kBrake);

    m_bottomEncoder = bottomMotor.getEncoder();

    m_ballDetect = new DigitalInput(Constants.kBeamBreakPort);    
  }

  public void motorInit(CANSparkMax motor, boolean invert){
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(Constants.kCurrentLimit);
    motor.setInverted(invert);

    //resets encoder
    encoderInit(motor.getEncoder());
  }

  private void encoderInit(RelativeEncoder encoder){
    //might add more stuff here later
    encoderReset(encoder);
  }
  
  public void encoderReset(RelativeEncoder encoder){
    encoder.setPosition(0);
  }
  
  public double getTopDistance(RelativeEncoder encoder){
    return encoder.getPosition();
  }

  //spinny motor
  public void load (double speed){
    bottomMotor.set(speed);
  }

  public boolean isBallPresent(){
    return !m_ballDetect.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
