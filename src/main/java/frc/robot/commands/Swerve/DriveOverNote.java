// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveOverNote extends PIDCommand {
  /** Creates a new PIDTurning. */
  public DriveOverNote(Swerve swerve, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(0.2, 0, 0.01),
        // This should return the measurement
        () -> -(light.getTY(null) + 4),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(output, 0), 0, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1.5);
    addRequirements(swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
