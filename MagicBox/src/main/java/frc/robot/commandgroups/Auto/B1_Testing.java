// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MultiSubsystem.AutoForwardPID;
import frc.robot.commands.MultiSubsystem.TurnInPlacePID;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class B1_Testing extends SequentialCommandGroup {
  /** Creates a new B1_A. */
  private final Drivetrain m_Drivetrain;

  public B1_Testing(Drivetrain subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Drivetrain = subsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // Score
      // new AutoScoreSetup(m_PinkArm, m_arm, m_gripper, pinkArmConstants.kMidAngleCone, TelescopingConstants.MidExtendCone),
      // new Dunk(m_PinkArm, m_arm, m_gripper),
      // new EjectItem(m_gripper),
      // new ExtendVal(5, m_arm),
      // new AutoForwardPID(-50 , m_Drivetrain),
      new AutoForwardPID(-3.4 , m_Drivetrain),
      new TurnInPlacePID(-12.11, m_Drivetrain),
      new AutoForwardPID(-0.6 , m_Drivetrain),
      new TurnInPlacePID(-179.7, m_Drivetrain),
      // new ParallelAutoScoreSetup(m_PinkArm, m_arm, m_gripper, pinkArmConstants.kLowAngleCube,TelescopingConstants.LowExtendCube),
      new AutoForwardPID(0.6 , m_Drivetrain)
      ); // 125.37, 114.26, -55.46,-59
  }
}
