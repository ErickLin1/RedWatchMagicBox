// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.NewLights;


public class NewChangeLEDColor extends Command {
  private final NewLights m_newlight;
  private final int m_Red;
  private final int m_Green;
  private final int m_Blue;
  

  /** Creates a new ChangeLEDColor. */
  public NewChangeLEDColor(NewLights light, int Red, int Green, int Blue) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_newlight = light;
      m_Red = Red;
      m_Green = Green;
      m_Blue = Blue;
      addRequirements(m_newlight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Sets lights to given color with RGB values
      m_newlight.setGiven(m_Red, m_Green, m_Blue);
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
