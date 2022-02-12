// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.PneumaticSolenoid.*;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Compressor airCompressor;
  private DoubleSolenoid solMotor;
  private DoubleSolenoid solMotor2;

  public Climber() {
    airCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);  //Digtial I/O,Relay
    //airCompressor = new Compressor(0, PneumaticsModuleType.REVPH);

    airCompressor.enableDigital();    
    
    solMotor = new DoubleSolenoid(PneumaticType, solMotorPort, solMotorPort + 1);
    solMotor2 = new DoubleSolenoid(PneumaticType, solMotorPort2, solMotorPort2 + 1);
  }

  /** Turns solenoid off and on. */
  public void toggleSolenoid() {
    if (solMotor.get().equals(Value.kForward)) {
      solMotor.set(Value.kReverse);
      solMotor2.set(Value.kReverse);
    } else {
      solMotor.set(Value.kForward);
      solMotor2.set(Value.kForward);
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
