// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;


public class ChangeLEDColor extends CommandBase {
  private final Lights m_light;
  private final double m_R;
  private final double m_G;
  private final double m_B;

  /** Creates a new ChangeLEDColor. */
  public ChangeLEDColor(Lights light, double R, double G, double B) {
    // Use addRequirements() here to declare subsystem dependencies.
      m_light = light;
      m_R = R;
      m_G = G;
      m_B = B;
      addRequirements(m_light);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_light.setToValue();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
