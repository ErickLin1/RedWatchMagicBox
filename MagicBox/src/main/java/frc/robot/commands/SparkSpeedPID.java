// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DrivetrainSparks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SparkSpeedPID extends PIDCommand {
  /** Creates a new SparkSpeedPID. */
  public SparkSpeedPID(DrivetrainSparks drivetrain, double speed) {
    super(
        // The controller that the command will use
        new PIDController(0, 0.001, 0),
        // This should return the measurement
        () -> drivetrain.getLeftSpeed(),
        // This should return the setpoint (can also be a constant)
        () -> speed,
        // This uses the output
        output -> {
          // Use the output here
          drivetrain.setLeftSpeed(MathUtil.clamp(output, -1, 1));;
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
