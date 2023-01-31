// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.Lights;
import static frc.robot.Constants.LightConstants.*;

public class CheckObjectColor extends CommandBase {
  public final ColorDetection m_colorDetect;
  public final Lights m_lights;

  /** Creates a new CheckObjectColor. */
  public CheckObjectColor(ColorDetection colorDetect, Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_colorDetect = colorDetect;
    m_lights = lights;
    addRequirements(colorDetect, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lights.setDisabledColor();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_colorDetect.m_detectedColor.green > m_colorDetect.m_detectedColor.blue && m_colorDetect.proximity >= 80)
      m_lights.setGiven(kPurpleCube);
    
    else if (m_colorDetect.m_detectedColor.blue > m_colorDetect.m_detectedColor.green && m_colorDetect.m_detectedColor.blue - m_colorDetect.m_detectedColor.green >= 200 && m_colorDetect.proximity < 120 && m_colorDetect.proximity > 30)
      m_lights.setGiven(kYellowCone);

    else
      m_lights.setDisabledColor();

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
