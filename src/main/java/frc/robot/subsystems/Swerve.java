package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_SwerveMods;
    public Pigeon2 m_gyro;
    public double m_slowDrive = 1;

    public Swerve() {
        m_gyro = new Pigeon2(SwerveConstants.pigeonID);
        m_gyro.getConfigurator().apply(new Pigeon2Configuration());
        m_gyro.setYaw(0);

        m_SwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        m_swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.75, 1.25, 0),
                new PIDConstants(2.75, 1.25, 0),
                SwerveConstants.maxSpeed,
                SwerveConstants.driveBaseRadius,
                new ReplanningConfig()
            ),
            () -> {
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                    return true;
                } else {
                    return false;
                }
            },
            this
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * m_slowDrive, 
                                    translation.getY() * m_slowDrive, 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * m_slowDrive, 
                                    translation.getY() * m_slowDrive, 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule mod : m_SwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : m_SwerveMods){
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_SwerveMods){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_SwerveMods){
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        // speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void futurePose(Pose2d pose) {
        
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValue());
    }

    public void setSpeedReducer(double reduce) {
        m_slowDrive = reduce;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : m_SwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        m_swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : m_SwerveMods){
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}