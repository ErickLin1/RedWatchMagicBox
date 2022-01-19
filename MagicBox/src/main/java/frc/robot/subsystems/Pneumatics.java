// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Pneumatics extends CommandBase {
  //private AxisCamera camera;                       //defines Axis Camera
  private DoubleSolenoid solPickUp;
  private DoubleSolenoid solPunch;                             //defines solenoids
  private DoubleSolenoid solHab;
  private DoubleSolenoid solArm;
  private DoubleSolenoid solCargoPickUp;
  private DoubleSolenoid solShoot;
  private Compressor airCompressor;

  public Pneumatics() {
      airCompressor = new Compressor(Constants.PneumaticType);  //Digtial I/O,Relay
      airCompressor.enableDigital();                        // Start the air compressor

      solPickUp = new DoubleSolenoid(Constants.PneumaticType, Constants.SOL_PICKUP_PORT, Constants.SOL_PICKUP_PORT + 1); // Solenoid ports
      solPunch = new DoubleSolenoid(Constants.PneumaticType, Constants.SOL_PUNCH_PORT, Constants.SOL_PUNCH_PORT + 1);  // What are the forward and reverse channel ports?
      solHab = new DoubleSolenoid(Constants.PneumaticType, Constants.SOL_HAB_PORT, Constants.SOL_HAB_PORT + 1); // Check if this works or not, updated for 2022
      solArm = new DoubleSolenoid(Constants.PneumaticType, Constants.SOL_ARM_PORT, Constants.SOL_ARM_PORT + 1);
  }

  public void pickupHatch(boolean out) {
      solPickUp.set(out ? Value.kForward : Value.kReverse);
  }

  public Value getPickupHatch() {
      return solPickUp.get();
  }

  public void punchHatch(boolean out) {
      solPunch.set(out ? Value.kForward : Value.kReverse);

  }

  public void outArm(boolean out) {
      solArm.set(out ? Value.kForward : Value.kReverse);
  }

  public void habIn(boolean out) {
      solHab.set(out ? Value.kForward : Value.kReverse);
  }

  public Value getPunchHatch() {
      return solPunch.get();
  }

  public void toggleArm() {
      if (solArm.get().equals(Value.kForward)) {
          solArm.set(Value.kReverse);
          return;
      }
      solArm.set(Value.kForward);

  }

  public void toggleGrab() {
      if (solPickUp.get().equals(Value.kForward)) {
          solPickUp.set(Value.kReverse);
          return;
      }
      solPickUp.set(Value.kForward);
  }

  public void togglePunch() {
      if (solPunch.get().equals(Value.kForward)) {
          solPunch.set(Value.kReverse);
          return;
      }
      solPunch.set(Value.kForward);
  }

  public Value getArm() {
      return solPickUp.get();
  }

  public void toggleHab() {
      if (solHab.get().equals(Value.kForward)) {
          solHab.set(Value.kReverse);
          return;
      }
      solHab.set(Value.kForward);
  }

  public Value getHab() {
      return solHab.get();
  }

  public void disableCompressor() {
      airCompressor.disable();
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
