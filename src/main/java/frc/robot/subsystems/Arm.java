// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.kArmID, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;

  private GenericEntry m_armAngle;
  private ShuffleboardTab m_tab;
  private GenericEntry m_armCurrent;

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(10000, 250)
          )
        );

    m_armMotor.setInverted(false);
    m_encoder = m_armMotor.getEncoder();
    m_encoder.setPositionConversionFactor((125 * (32 / 12)) / 360);
    m_encoder.setPositionConversionFactor(1);
    m_encoder.setPosition(0);

    m_controller.setTolerance(1.5);

    m_tab = Shuffleboard.getTab("Main");
    m_armAngle = m_tab.add("Arm Angle", getAngle()).withWidget(BuiltInWidgets.kTextView).getEntry();
    m_armCurrent = m_tab.add("Arm Current", getCurrent()).withWidget(BuiltInWidgets.kGraph).withSize(3, 2).withPosition(5, 1).getEntry();

    setAngle(0);
    enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_armMotor.setVoltage(output);
  }

  public double getCurrent() {
    return m_armMotor.getOutputCurrent();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  public void setAngle(double angle) {
    setGoal(angle);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public void up() {
    m_armMotor.set(0.5);
  }

  public void down() {
    m_armMotor.set(-0.5);
  }

  public double distanceToArmAngle(double distance) {
    return distance;
  }

  public double shooterAngleToArmAngle(double angle) {
    return angle;
  }

  public void periodic() {
    super.periodic();
    // System.out.println("At setpoint: " + m_controller.atSetpoint());
    m_armAngle.setDouble(getAngle());
    m_armCurrent.setDouble(getCurrent());
  }
}
