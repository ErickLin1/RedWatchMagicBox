// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.motor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PotExtend extends CommandBase {
  /** Creates a new PotExtend. */
  private final motor m_Arm;
  private final double potDistance;

  public PotExtend(double distance, motor armMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = armMotor;
    potDistance = distance;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setSpeed(1);
    if (m_Arm.getPotValue() >= potDistance){
      m_Arm.stopMotor();
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_Arm.getPotValue() >= potDistance){
      return true;
    }
    return false;
  }
}
