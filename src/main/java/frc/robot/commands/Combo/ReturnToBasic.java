// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.WaitForArmAngle;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnToBasic extends ParallelDeadlineGroup {
  /** Creates a new ReturnToBasic. */
  public ReturnToBasic(Arm arm, Shooter shooter, Intake intake, Feeder feeder) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(
      new WaitForArmAngle(() -> ArmConstants.kArmHomeAngle, arm),
      new SetShooterSpeed(0, shooter),
      new SetFeederSpeed(0, feeder),
      new StopIntake(intake)
    );
    // addCommands(new FooCommand(), new BarCommand());
  }
}
