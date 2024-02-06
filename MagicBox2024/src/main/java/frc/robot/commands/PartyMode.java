// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.NewLights;

public class PartyMode extends Command {
  /** Creates a new AskForPiece. */
  public NewLights m_NewLights;
  boolean strobe = false;

  public PartyMode(NewLights lights, boolean val) {

    // Use addRequirements() here to declare subsystem dependencies.
    m_NewLights = lights;
    strobe = val;
    addRequirements(m_NewLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Sets lights to epilepsy if strobe is true
    if(strobe == true){
      m_NewLights.epilepsy();
    }
    // Sets lights to rainbow party mode if strobe is false
    else{
      m_NewLights.partyMode();
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