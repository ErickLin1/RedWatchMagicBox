// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Lights;

public class CheckObjectColor extends CommandBase {
  public final Gripper m_gripper;
  public final Lights m_lights;

  /** Creates a  CheckObjectColor. */
  public CheckObjectColor(Gripper gripper, Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gripper = gripper;
    m_lights = lights;
    addRequirements(gripper, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets lights back to default
    m_lights.setDefault();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets lights to detected cube if Color sensor detects purple and the detected color is closer than 2 inches
    if (m_gripper.isPurple())
      m_lights.setCube();
    
    // Sets lights to detected cone if Color sensor detects yellow and the detected color is between 1.5 and 4.5 inches
    else if (m_gripper.isYellow())
      m_lights.setCone();

    //If color sensor does not detect required values, set lights back to default
    else
      m_lights.setDefault();

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
