// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ColorDetection;
import frc.robot.subsystems.NewLights;
import static frc.robot.Constants.LightConstants.*;

public class NewCheckObjectColor extends Command {
  public final ColorDetection m_colorDetect;
  public final NewLights m_newlights;

  /** Creates a new CheckObjectColor. */
  public NewCheckObjectColor(ColorDetection colorDetect, NewLights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_colorDetect = colorDetect;
    m_newlights = lights;
    addRequirements(colorDetect, lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Resets lights back to default
    m_newlights.setDefault();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If color sensor detects
    if(m_colorDetect.detect){
      // Sets lights to detected cube if Color sensor detects purple and the detected color is closer than 2 inches
      if (m_colorDetect.m_detectedColor.green > m_colorDetect.m_detectedColor.blue && m_colorDetect.proximity >= 80)
        m_newlights.setCube();
      
      // Sets lights to detected cone if Color sensor detects yellow and the detected color is between 1.5 and 4.5 inches
      else if (m_colorDetect.m_detectedColor.blue > m_colorDetect.m_detectedColor.green && m_colorDetect.m_detectedColor.blue - m_colorDetect.m_detectedColor.green >= 200 && m_colorDetect.proximity < 120 && m_colorDetect.proximity > 30)
        m_newlights.setCone();

      //If color sensor does not detect required values, set lights back to default
      else
        m_newlights.setDefault();
    }

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
