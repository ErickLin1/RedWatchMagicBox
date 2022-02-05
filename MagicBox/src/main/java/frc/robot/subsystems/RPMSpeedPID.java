// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class RPMSpeedPID extends PIDSubsystem {
  /** Creates a new RPMSpeedPID. */
  private final CANSparkMax m_motor;
  public RPMSpeedPID(CANSparkMax motor) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    m_motor = motor;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_motor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_motor.getEncoder().getVelocity();
  }
}
