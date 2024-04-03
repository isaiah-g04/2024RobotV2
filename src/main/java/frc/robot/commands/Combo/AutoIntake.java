// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Blinkin.BlinkLimelight;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Feeder.WaitForNoNote;
import frc.robot.commands.Feeder.WaitForNote;
import frc.robot.commands.Intake.IdleIntake;
import frc.robot.commands.Intake.IdleOuttake;
import frc.robot.commands.Intake.SimpleIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(Feeder feeder, Intake intake, Arm arm, Shooter shooter, Limelight light) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitForNote(feeder), 
        new SimpleIntake(intake),
        new SetFeederSpeed(10, feeder)
      ),
      new ParallelDeadlineGroup(
        new WaitForNoNote(feeder), 
        new SetFeederSpeed(-2, feeder),
        new IdleOuttake(intake)
      ),
      new SetFeederSpeed(0, feeder),
      new ParallelCommandGroup(
        new BlinkLimelight(light),
        new IdleIntake(intake),
        new SetArmAngle((ArmConstants.kIdleArmAngle), arm),
        new SetShooterSpeed(ShooterConstants.kIdleSpeedRPS, shooter),
        new StopIntake(intake)
      )
    );
  }
}
