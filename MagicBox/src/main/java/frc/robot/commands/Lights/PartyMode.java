// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class PartyMode extends CommandBase {
  /** Creates a new AskForPiece. */
  public Lights m_lights;
  boolean strobe = false;

  public PartyMode(Lights lights, boolean val) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_lights = lights;
    strobe = val;
    addRequirements(m_lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets lights to epilepsy if strobe is true
    if(strobe == true){
      m_lights.epilepsy();
    }
    // Sets lights to rainbow party mode if strobe is false
    else{
      m_lights.partyMode();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}