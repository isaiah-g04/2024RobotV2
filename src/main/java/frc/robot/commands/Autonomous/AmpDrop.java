// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.Arm.WaitForArmAngle;
import frc.robot.commands.Combo.ReturnToBasic;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Feeder.WaitForNoNote;
import frc.robot.commands.Feeder.WaitForNote;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpDrop extends SequentialCommandGroup {
  /** Creates a new AmpDrop. */
  public AmpDrop(Arm arm, Intake intake, Shooter shooter, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new WaitForArmAngle(() -> ArmConstants.kAmpAngle, arm), 
        new SetShooterSpeed(10, shooter)
      ),
      new ParallelDeadlineGroup(
        new WaitForNote(feeder), 
        new SetFeederSpeed(10, feeder)
      ),
      new ParallelDeadlineGroup(
        new WaitForNoNote(feeder), 
        new SetFeederSpeed(10, feeder)
      ),
      new ReturnToBasic(arm, shooter, intake, feeder)
    );
  }
}
