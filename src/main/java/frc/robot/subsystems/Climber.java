// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_armClimber = new CANSparkMax(ClimberConstants.kArmClimberID, MotorType.kBrushless);
  private final CANSparkMax m_hookClimber = new CANSparkMax(ClimberConstants.kHookClimberID, MotorType.kBrushless);

  private final RelativeEncoder m_armEncoder = m_armClimber.getEncoder();
  private final RelativeEncoder m_hookEncoder = m_hookClimber.getEncoder();

  private final SparkPIDController m_armPID;
  private final SparkPIDController m_hookPID;

  private boolean m_isEnable = false;

  private double m_armSetpoint = 0;
  private double m_hookSetpoint = 0;

  /** Creates a new Climber. */
  public Climber() {
    
    m_armClimber.setInverted(false);
    m_hookClimber.setInverted(true);

    m_armClimber.setIdleMode(IdleMode.kBrake);
    m_hookClimber.setIdleMode(IdleMode.kBrake);

    m_armPID = m_armClimber.getPIDController();
    m_hookPID = m_hookClimber.getPIDController();

    m_armEncoder.setPosition(0);
    m_hookEncoder.setPosition(0);

    m_armEncoder.setPositionConversionFactor(0.05);
    m_hookEncoder.setPositionConversionFactor(0.05);

    setupPID();

    setHookSetpoint(0);
    setArmSetpoint(0);

    enable();
  }
  public void resetArmEncoder() {
    m_armEncoder.setPosition(0);    
  }

  public void resetHookEncoder() {
    m_hookEncoder.setPosition(0);
  }

  public void setupPID() {
    m_armPID.setP(0);
    m_armPID.setOutputRange(-1, 1);
    m_hookPID.setP(0);
    m_hookPID.setOutputRange(-1, 1);
  }

  public void enable() {
    m_isEnable = true;
  }

  public void disable() {
    m_isEnable = false;
  }

  public void setArmSetpoint(double armSetpoint) {
    m_armSetpoint = armSetpoint;
  }
  public double getArmSetpoint() {
    return m_armSetpoint;
  }

  public void setHookSetpoint(double hookSetpoint) {
    m_hookSetpoint = hookSetpoint;
  }

  public double getHookSetpoint() {
    return m_hookSetpoint;
  }
  public void raiseArms() {
    m_armClimber.set(0.75);
  }

  public void lowerArms() {
    m_armClimber.set(-0.75);
  }

  public void stopArms() {
    m_armClimber.stopMotor();
  }

  public void raiseHooks() {
    m_hookClimber.set(0.75);
  }

  public void lowerHooks() {
    m_hookClimber.set(-0.75);
  }

  public void stopHooks() {
    m_hookClimber.stopMotor();
  }

  public double getArmEncoder() {
    return m_armEncoder.getPosition();
  }

  public double getHookEncoder() {
    return m_hookEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_isEnable) {
      m_armPID.setReference(m_armSetpoint, CANSparkMax.ControlType.kPosition);
      m_hookPID.setReference(m_armSetpoint, CANSparkMax.ControlType.kPosition);
    }
  }
}
