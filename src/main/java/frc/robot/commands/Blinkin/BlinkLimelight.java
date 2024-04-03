// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Blinkin;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;

public class BlinkLimelight extends Command {
  private final Timer m_timer;

  /** Creates a new BlinkLimelight. */
  public BlinkLimelight(Limelight light) {
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.setLEDMode_ForceBlink("limelight-shooter");
    LimelightHelpers.setLEDMode_ForceBlink("limelight-intake");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOff("limelight-shooter");
    LimelightHelpers.setLEDMode_ForceOff("limelight-intake");
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > 0.75) {
      return true;
    } else {
      return false;
    }
  }
}
