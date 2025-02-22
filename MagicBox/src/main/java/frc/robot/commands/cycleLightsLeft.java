// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class cycleLightsLeft extends CommandBase {

  private final Lights m_light;
  /** Creates a new cycleLightsLeft. */
  public cycleLightsLeft(Lights light) {
    m_light = light;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentColor = m_light.getCurrentLights();
    if (currentColor != -0.99) {
      m_light.setGiven(currentColor - 0.02);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}