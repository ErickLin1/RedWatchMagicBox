// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lights;

public class DiscoDiscoWhenBalance extends CommandBase {
  public final Drivetrain m_drivetrain;
  public final Lights m_lights;

  /** Creates a new DiscoDiscoWhenBalance. */
  public DiscoDiscoWhenBalance(Lights lights, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_lights = lights;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Math.abs(Constants.BalanceConstants.kBalancedBeamAngle - m_drivetrain.getPitch()) < Constants.BalanceConstants.kBalancedThreshold){
      m_lights.partyMode();
    }
    else {
      m_lights.setDefault();
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
