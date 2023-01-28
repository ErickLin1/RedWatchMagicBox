package frc.robot.subsystems;

import static frc.robot.Constants.TelescopingConstants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class TelescopingArm extends SubsystemBase {
    
public final CANSparkMax m_ArmExtend;
public final CANSparkMax m_ArmPivot;

public final RelativeEncoder m_ArmEncoder;
public final RelativeEncoder m_pivotArmEncoder;

// /private static AHRS m_ahrs;

/**
 * Controls Telescoping Mechanism
 */

 public TelescopingArm() {
    
    m_ArmExtend = new CANSparkMax(kArmExtendPort, MotorType.kBrushless);
    m_ArmPivot = new CANSparkMax(kArmPivotPort, MotorType.kBrushless);

    // setMotor(motor, INVERSE);
    setMotor(m_ArmExtend, true);
    setMotor(m_ArmPivot, true);


    m_ArmEncoder = m_ArmExtend.getEncoder();

    m_pivotArmEncoder = m_ArmPivot.getEncoder();
    
    positionEncoderInit(m_ArmEncoder);
    pivotEncoderInit(m_pivotArmEncoder);
  /*
    try {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } 
    catch (RuntimeException ex){
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }*/
  }

  public void changeMode(String mode) {

  }

  public void turnMotor(CANSparkMax motor, boolean inverse) {
    if (inverse) {
      motor.set(-0.1);
    }
    else {
      motor.set(0.1);
    }
  }

  private void positionEncoderInit(RelativeEncoder encoder) {
    encoder.setPositionConversionFactor(kDistancePerRevolution);

    encoderReset(encoder);
  }

  private void pivotEncoderInit(RelativeEncoder encoder) {
    encoder.setPositionConversionFactor(kAnglePerRevolution);
  }

  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }


  public double getArmDistance() {
    return -m_ArmEncoder.getPosition();
  }

  public double getRightPivot() {
    return -m_pivotArmEncoder.getPosition();
  }

  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }
/* 
  public double getGyroAngle(){
    return m_ahrs.getAngle();
  }

  public void resetGyroAngle(){
    m_ahrs.reset();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }
 }

