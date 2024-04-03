// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_frontIntake = new CANSparkMax(IntakeConstants.kFrontIntakeID, MotorType.kBrushless);
  private final CANSparkMax m_backIntake = new CANSparkMax(IntakeConstants.kBackIntakeID, MotorType.kBrushless);

  private final RelativeEncoder m_frontEncoder;
  private final RelativeEncoder m_backEncoder;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_intakeCurrent;
  private final GenericEntry m_intakeRunning;
  private final GenericEntry m_touchingNote;

  /** Creates a new Intake. */
  public Intake() {
    m_frontIntake.setInverted(false);
    m_backIntake.setInverted(true);

    m_frontEncoder = m_frontIntake.getEncoder();
    m_backEncoder = m_backIntake.getEncoder();

    m_frontEncoder.setVelocityConversionFactor((Math.PI * IntakeConstants.kWheelDiameterIN) / (12 * 60 * IntakeConstants.kGearboxRatio));
    m_backEncoder.setVelocityConversionFactor((Math.PI * IntakeConstants.kWheelDiameterIN) / (12 * 60 * IntakeConstants.kGearboxRatio));

    m_intakeCurrent = m_tab.add("Intake Current", m_frontIntake.getOutputCurrent()).getEntry();
    m_intakeRunning = m_tab.add("Intake Running", intakeRunning()).getEntry();
    m_touchingNote = m_tab.add("Touching Note", currentDetectedNote()).getEntry();
  }

  public void runMotors(double speed) {
    m_frontIntake.set(speed);
    m_backIntake.set(speed);
  }

  public double motorCurrent() {
    return m_frontIntake.getOutputCurrent();
  }

  public boolean currentDetectedNote() {
    if(motorCurrent() >= 11) {
      return true;
    } else {
      return false;
    }
  }

  public boolean intakeRunning() {
    if (m_frontIntake.getOutputCurrent() > 1) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      m_intakeCurrent.setDouble(motorCurrent());
      m_intakeRunning.setBoolean(intakeRunning());
      m_touchingNote.setBoolean(currentDetectedNote());
    }
  }
