// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Feeder.WaitForNoNote;
import frc.robot.commands.Feeder.WaitForNote;
import frc.robot.commands.Intake.IdleOuttake;
import frc.robot.commands.Intake.SimpleIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Swerve.DriveOverNote;
import frc.robot.commands.Swerve.PIDTranslation;
import frc.robot.commands.Swerve.PIDTurningIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoAdjustIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoAdjustIntake(Feeder feeder, Intake intake, Arm arm, Shooter shooter, Limelight light, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDTranslation(swerve, light),
      new DriveOverNote(swerve, light),
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
      new StopIntake(intake),
      new SetFeederSpeed(0, feeder)
    );
  }
}
