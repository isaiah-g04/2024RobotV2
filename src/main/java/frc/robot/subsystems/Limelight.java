// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  public GenericEntry m_distance;
  private ShuffleboardTab m_tab;

  public double m_goodDistance = getDistance();

  /** Creates a new Limelight. */
  public Limelight() { //TODO: name limelights and add intake methods
    setCorrectTarget();
    m_tab = Shuffleboard.getTab("Main");
    m_distance = m_tab.add("Distance", getDistance()).withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  public double getDistance() {
    // System.out.println((LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("limelight-shooter"))));
    return (LimelightConstants.kGoalHeightMeters - LimelightConstants.kLimelightLensHeightMeters) / Math.tan(LimelightConstants.kMountAngleRadians + Units.degreesToRadians(LimelightHelpers.getTY("limelight-shooter")));
  }

  public double getTargetArmAngle() {
    return (15.2 + (16.7 * getDistance()) + (-1.68 * Math.pow(getDistance(), 2)));
  }

  public double getTargetRPM() {
    // System.out.println((65 + (5 * getDistance())));
    return (65 + (5 * getDistance()));
  }

  public double getTX(String limelight) {
    return LimelightHelpers.getTX(limelight);
  }

  public double getTY(String limelight) {
    return LimelightHelpers.getTY(limelight);
  }

  private void setCorrectTarget() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPriorityTagID("limelight-shooter", 7);
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      LimelightHelpers.setPriorityTagID("limelight-shooter", 4);
    } else {
      // DriverStation.reportError("Did not get alliance to setup Limelight.", true);
      System.out.println("Did not get alliance to setup Limelight.");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_distance.setDouble(getDistance());
  }
}
