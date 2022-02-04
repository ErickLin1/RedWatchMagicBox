// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Creates a subsystem for the beam break. */
public class Beambreak extends SubsystemBase {
  private final DigitalInput input;

  public Beambreak() {
    input = new DigitalInput(Constants.BeambreakPort);
  }

  /**
   * 
   * @return Returns a boolean whether the beam is connected or not.
   */
  public boolean get() {
    return input.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
