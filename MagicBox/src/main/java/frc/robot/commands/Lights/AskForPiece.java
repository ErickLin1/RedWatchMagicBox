// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;

public class AskForPiece extends CommandBase {
  /** Creates a new AskForPiece. */
  public Lights m_lights;
  public Gripper m_gripper;
  private boolean m_cube;

  public AskForPiece(Lights lights, Gripper gripper, boolean cube) {
    m_lights = lights;
    m_gripper = gripper;
    m_cube = cube;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_cube){
      m_lights.askForCube();
    }
    else if (!m_cube){
      m_lights.askForCone();
    }
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
    return false;
  }
}
