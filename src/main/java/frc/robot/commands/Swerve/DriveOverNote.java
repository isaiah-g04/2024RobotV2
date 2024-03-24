// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveOverNote extends PIDCommand {
  private final Timer m_timer = new Timer();
  private final Limelight lite;

  /** Creates a new PIDTurning. */
  public DriveOverNote(Swerve swerve, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0), //TODO: tune this
        // This should return the measurement
        (() -> light.getTY("limelight-intake") + 4),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          System.out.println(output);
          swerve.drive(new Translation2d(output, 0), 0, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_timer.start();
    getController().setTolerance(0.25);
    addRequirements(swerve);
    lite = light;
  }

  // Returns true when the command should end.%
  @Override
  public boolean isFinished() {
    // System.out.println((lite.getTY("limelight-intake") + 10));
    if (m_timer.get() < 0.5) {
      return false;
    } else if(m_timer.get() > 1) {
      return true;
      // return false;
    } else {
      return getController().atSetpoint();
    }
  }
}
