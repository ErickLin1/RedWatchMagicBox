// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OneMotor;

public class RunMotor extends CommandBase {
  /** Creates a new RunMotor. */
  private final OneMotor m_Motor;
  double m_speed;
  public RunMotor(OneMotor oneMotor, double speed) {
    m_Motor = oneMotor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Motor);
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Motor.setSpeed(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
