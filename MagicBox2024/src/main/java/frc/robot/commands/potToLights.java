// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MeasuringPotentiometer;
import frc.robot.subsystems.Lights;

public class potToLights extends Command {
  private final Lights m_lights;
  private final MeasuringPotentiometer m_pot;
  /** Creates a new potToLights. */
  public potToLights(MeasuringPotentiometer pot, Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lights = lights;
    m_pot = pot;

    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lights.setGiven(m_pot.pot_value);
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
