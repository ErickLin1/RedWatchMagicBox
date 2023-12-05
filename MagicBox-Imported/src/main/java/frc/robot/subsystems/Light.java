// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class Light extends SubsystemBase {
  /** Creates a new Light. */

  private final Spark ledController;

  public Light() {
    ledController = new Spark(Constants.kBlinkinDriverPort);
  }

  public void turnLEDOn() {
    ledController.set(Constants.kRed);
  }
  
  public void turnLEDOff() {
    ledController.set(Constants.kOff);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
