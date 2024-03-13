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

public class PIDTranslation extends PIDCommand {
  private final Timer m_timer = new Timer();

  /** Creates a new PIDTurning. */
  public PIDTranslation(Swerve swerve, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(0.2, 0.1, 0), //TODO: tune this
        // This should return the measurement
        () -> light.getTX("intake"),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(output, 0), 0, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_timer.start();
    getController().setTolerance(1.5);
    addRequirements(swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() < 0.3) {
      return false;
    } else if(m_timer.get() > 1) {
      return true;
    } else {
      return getController().atSetpoint();
    }
  }
}
