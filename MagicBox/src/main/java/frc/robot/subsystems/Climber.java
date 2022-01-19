// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Compressor airCompressor;
  private DoubleSolenoid solMotor;
  
  public Climber() {
    airCompressor = new Compressor(0, Constants.PneumaticType);  //Digtial I/O,Relay
    airCompressor.enableDigital();    
    
    solMotor = new DoubleSolenoid(Constants.PneumaticType, Constants.solMotorPort, Constants.solMotorPort + 1);
  }

  public void toggleSolenoid() {
    if (solMotor.get().equals(Value.kForward)) {
      solMotor.set(Value.kReverse);
    } else {
      solMotor.set(Value.kForward);
    }
  }

  public void disableCompressor() {
    airCompressor.close();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
