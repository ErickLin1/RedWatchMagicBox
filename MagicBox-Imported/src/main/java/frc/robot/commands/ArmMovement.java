// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.motor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmMovement extends CommandBase {
  /** Creates a new ArmMovement. */
  private final motor m_Arm;
  private double m_speed;


  public ArmMovement(double speed, motor armMotor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = armMotor;
    m_speed = speed;
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
