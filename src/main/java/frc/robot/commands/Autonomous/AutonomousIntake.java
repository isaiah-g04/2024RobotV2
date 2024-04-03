// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Combo.ReturnToBasic;
import frc.robot.commands.Feeder.SetFeederSpeed;
import frc.robot.commands.Intake.SimpleIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutonomousIntake extends Command {
  private final Intake m_intake;
  private final Feeder m_feeder;
  private final Shooter m_shooter;
  private final Arm m_arm;
  private final Timer m_timer = new Timer();


  /** Creates a new AutonomousIntake. */
  public AutonomousIntake(Intake intake, Feeder feeder, Shooter shooter, Arm arm, Timer timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_feeder = feeder;
    m_shooter = shooter;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
      // System.out.println("Current Check Starting"),
      new SimpleIntake(m_intake),
      // System.out.println("Step 1"),
     new SetFeederSpeed(10, m_feeder)
      // System.out.println("Step 2")
    );
System.out.println("Past Sequential Group");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Done bro");
    new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_timer.get() < 1) {
    //   return false; 
    // } else if (m_intake.currentDetectedNote() == true ) {
    //   return true;
    // } 
    // else if (m_intake.currentDetectedNote() == false && m_timer.get() >= 3) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
