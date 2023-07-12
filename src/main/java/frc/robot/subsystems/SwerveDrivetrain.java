// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {
  private SwerveModule[] modules;
  private SwerveModulePosition[] modulePositions;

  /*
   * The SwerveDriveKinematics class is a useful tool that converts
   * between a ChassisSpeeds object and several SwerveModuleState 
   * objects, which contains velocities and angles for each swerve 
   * module of a swerve drive robot.
   */
  private SwerveDriveKinematics swerveKinematics;

  //Gyro for the robot
  private AHRS gyro;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);
    modules = new SwerveModule[4];
    modulePositions = new SwerveModulePosition[4];

    swerveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
