// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;

public class ProximitySensor2024 extends SubsystemBase {
  /* Initialize fields */
  private double lastDistRead;

  public static final double kHoldDistanceMillimeters = 1.0e3;

  private final MedianFilter m_filter = new MedianFilter(5);
  private final Ultrasonic m_ultrasonic;

  /** Creates a new ProximitySensor2024. */
  public ProximitySensor2024() {
    m_ultrasonic = new Ultrasonic(Constants.ProximitySensor24Constants.kUltrasonicPingPort,
        Constants.ProximitySensor24Constants.kUltrasonicEchoPort);

  }

  // returns current distance in meters
  public double getSensorDist() {
    return (m_ultrasonic.getRangeMM() / 1000);

  }

  // Returns true if the value returned is less than or equal to the distance
  public boolean withinDist(double dist) {
    return getSensorDist() <= dist;
  }

  public void setup() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double measurement = m_ultrasonic.getRangeMM();

  }
}
