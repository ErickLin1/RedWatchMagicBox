// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class NewChangeLEDColor extends CommandBase {
  private final Lights m_lights;
  private final int m_R;
  private final int m_G;
  private final int m_B;

  /** Creates a new ChangeLEDColor. */
  public NewChangeLEDColor(Lights lights, int r, int g, int b) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_lights = lights;
      m_R = r;
      m_G = g;
      m_B = b;
      addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Sets lights to given color with RGB values
      m_lights.setGiven(m_R, m_G, m_B);
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
