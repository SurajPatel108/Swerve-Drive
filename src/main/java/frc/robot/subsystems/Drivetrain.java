// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveModule FL;
  private final SwerveModule FR;
  private final SwerveModule BL;
  private final SwerveModule BR;

  private final Translation2d frontLeftLocation = ;  // make constants and add mod location
  private final Translation2d frontRightLocation = ;
  private final Translation2d backLeftLocation = ;
  private final Translation2d backRightLocation = ;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    FL = new SwerveModule(); // fl, turn motor
    FR = new SwerveModule(); // fr, turn motor
    BL = new SwerveModule(); // bl, turn motor
    BR = new SwerveModule(); // br, turn motor

    odometry = new SwerveDriveOdometry(
      kinematics,
      ahrs.getRotation2d(),
      new SwerveModulePosition[] {
        FL.getModulePosition(),
        FR.getModulePosition(),
        BL.getModulePosition(),
        BR.getModulePosition()
      },
      new Pose2d(0, 0, new Rotation2d())
    );

    ahrs.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed);

    FL.setDesiredState(swerveModuleStates[0]);
    FR.setDesiredState(swerveModuleStates[1]);
    BL.setDesiredState(swerveModuleStates[2]);
    BR.setDesiredState(swerveModuleStates[3]);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    odometry.update(ahrs.getRotation2d(), new SwerveModulePosition[] {
      FL.getModulePosition(),
      FR.getModulePosition(),
      BL.getModulePosition(),
      BR.getModulePosition()
    });
  }
}
