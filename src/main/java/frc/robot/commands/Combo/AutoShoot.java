// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.SetArmAngle;
import frc.robot.commands.Arm.WaitForArmAngle;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Feeder.WaitForNoNote;
import frc.robot.commands.Feeder.WaitForNote;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.commands.Shooter.WaitForShooterSpeed;
import frc.robot.commands.Swerve.PIDTurning;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Swerve swerve, Limelight light, Feeder feeder, Arm arm, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitForArmAngle(() -> light.getTargetArmAngle(), arm),
        new WaitForShooterSpeed(light.getTargetRPM(), shooter),
        new PIDTurning(swerve, light)
      ),
      new ParallelDeadlineGroup(
        new WaitForNote(feeder), 
        new SetFeederSpeed(10, feeder)
      ),
      new ParallelDeadlineGroup(
        new WaitForNoNote(feeder), 
        new SetFeederSpeed(10, feeder)
      ),
      new ParallelCommandGroup(
        new SetShooterSpeed(0, shooter),
        new SetFeederSpeed(0, feeder),
        new SetArmAngle(0, arm)
      )
    );
  }
}
