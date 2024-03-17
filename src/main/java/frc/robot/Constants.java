package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.2;

    public static final class OIConstants {
        public static final int kDriverController = 0;
        public static final int kOperatorController = 1;
    }

    public static final class ShooterConstants {
        public static final int kTopShooterID = 9;
        public static final int kBottomShooterID = 10;
        public static final double kShooterSpeedCloseRPS = 45;
        public static final double kShooterSpeedMidRPS = 65;
        public static final double kShooterSpeedFarRPS = 95;
        public static final double kShooterSpeedAmpRPS = 15;
        public static final double kShooterSpeedYeetRPS = 40;
    }

    public static final class FeederConstants {
        public static final int kFeederID = 11;
        public static final int kLaserCANID = 12;
        public static final int kFeederSpeedFPS = 10;
        public static final int kWheelDiameterIN = 2;
        public static final int kGearboxRatio = 12;
        public static final double kFeedingSpeedFPS = 10;
    }

    public static final class ArmConstants {
        public static final int kArmID = 13;
        public static final double kMaxAngle = 130;
        public static final double kAmpAngle = 100;
        public static final double kSpeakerAngle = 20;
        public static final double kSpeakerCloseAngle = 32;
        public static final double kSpeakerMidAngle = 50;
        public static final double kSpeakerFarAngle = 58.75;
        public static final double kArmHomeAngle = 0;
    public static final double kYeetAngle = 50; //TODO: Tune this number at Mason
    }

    public final class IntakeConstants {
        public static final int kFrontIntakeID = 14;
        public static final int kBackIntakeID = 15;
        public static final int kIntakeSpeedFPS = 4;
        public static final int kWheelDiameterIN = 2;
        public static final int kGearboxRatio = 16;
    }

    public static final class LimelightConstants {
        public static final double kMountAngleRadians = Units.degreesToRadians(24);
        public static final double kLimelightLensHeightMeters = Units.inchesToMeters(15.75);
        public static final double kGoalHeightMeters = Units.inchesToMeters(53.875);
    }

    public final class ClimberConstants {
        public static final int kArmClimberID = 16;
        public static final int kHookClimberID = 17;
    }

    public final class LEDConstants {
        public static final int kBlinkinPort = 1;
    }

    public final class SwerveConstants {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double driveBaseRadius = Math.sqrt((trackWidth * trackWidth) + (wheelBase * wheelBase));
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 15;
        public static final int angleStatorCurrnetLimit = 20;
        public static final int angleCurrentThreshold = 30;
        public static final double angleCurrentThresholdTime = 0.0;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 25;
        public static final int driveStatorCurrnetLimit = 50;
        public static final int driveCurrentThreshold = 40;
        public static final double driveCurrentThresholdTime = 0.0;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(17); //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(97.6464);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(154.5996);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(32.0800);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.1503);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
