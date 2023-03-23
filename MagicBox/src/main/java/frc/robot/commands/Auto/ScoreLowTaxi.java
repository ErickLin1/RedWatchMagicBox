// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.MultSubsystem.AutoForwardPID;
import frc.robot.commands.PivotArm.PivotPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.TelescopingArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/*
 * Scores a cone low, and goes outside the community.
 * Path is symmetrical wether on left or right side of the field, or on red or blue alliance.
 */
public class ScoreLowTaxi extends SequentialCommandGroup {
  /** Creates a new ScoreLowTaxi. */
  public ScoreLowTaxi(Drivetrain drivetrain, Gripper gripper, PivotArm pivotArm, TelescopingArm telescopingArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     // new SetupScore(m_PivotArm, m_TelescopingArm, Constants.pinkArmConstants.kLowAngleCube+10 , Constants.TelescopingConstants.LowExtendCube),
      new PivotPID(pivotArm, Constants.pinkArmConstants.kLowAngleCube+15),
      new WaitCommand(0.5),
      new AutoForwardPID(-4.2, drivetrain)
    );
  }
}
