package frc.robot.subsystems;

import static frc.robot.Constants.TelescopingConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingArm extends SubsystemBase {
    
public final CANSparkMax m_ArmExtend;// Calculates the potential of the control panel.
public final AnalogPotentiometer pot;
public double pot_val;
public double offset = 0;

public final RelativeEncoder m_ArmEncoder;

  /**
   * Controls Telescoping Mechanism
   */
  public TelescopingArm() {
    m_ArmExtend = new CANSparkMax(kArmExtendPort, MotorType.kBrushless);

    setMotor(m_ArmExtend, true);
    m_ArmEncoder = m_ArmExtend.getEncoder();
    positionEncoderInit(m_ArmEncoder);

    pot = new AnalogPotentiometer(1);

  }

  // Gets the distance from this value to this value.
  public double getDistance(){
    return pot_val;
  }

  // Turns the motor on or off.
  public void turnMotor(CANSparkMax motor, boolean inverse) {
    if (inverse) {
      motor.set(-ArmSpeed);
    }

    else {
      motor.set(ArmSpeed);
    }
  }

  // Initializes the position encoder.
  private void positionEncoderInit(RelativeEncoder encoder) {
    encoder.setPositionConversionFactor(kDistancePerRevolution);

    encoderReset(encoder);
  }

  // Resets the encoder to its initial position.
  public void encoderReset(RelativeEncoder encoder) {
    encoder.setPosition(0);
  }

  // Returns the distance between the encoding and the arm.
  public double getArmDistance() {
    return -m_ArmEncoder.getPosition();
  }

  // Sets the motor.
  public void setMotor(CANSparkMax motor, boolean inverse) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(inverse);
  }

  // Periodically calculates the value of the pot.
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pot_val = ((pot.get())*50)-offset;
  }
}
