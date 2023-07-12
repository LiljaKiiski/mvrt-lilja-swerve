// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SwerveDrivetrain extends SubsystemBase {

  //Modules and positions
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
  private double gyroOffset_deg = 0;

  //Representation of field
  private Field2d field;
  public boolean fieldOriented = true;

  //PID Controllers
  public PIDController xController;
  public PIDController yController;
  public PIDController rotationControllerFeedBack;
  /*
   * Basically, the profiled PID controller is a normal PID 
   * controller that follows a trapezoidal motion profile. 
   * Assuming it is relatively far from the target, the 
   * controller would accelerate to max velocity (as supplied
   *  in constraints), maintain max velocity, and then decelerate
   *  until it hits the goal
   */
  public ProfiledPIDController thetaController;
  private TrajectoryConfig trajectoryConfig;

  /** Creates a new SwerveDrivetrain. */
  public SwerveDrivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);

    //Kinematics
    swerveKinematics = new SwerveDriveKinematics(
      Constants.SwerveDrivetrain.m_frontRightLocation, 
      Constants.SwerveDrivetrain.m_frontLeftLocation, 
      Constants.SwerveDrivetrain.m_backLeftLocation, 
      Constants.SwerveDrivetrain.m_backRightLocation);

    //Positions of modules
    modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    //Modules themselves
    modules = new SwerveModule[4];

    modules[0] = new SwerveModule(
      "FrontRight",
      Constants.SwerveDrivetrain.m_frontRightDriveID,
      Constants.SwerveDrivetrain.m_frontRightTurnID,
      Constants.SwerveDrivetrain.m_frontRightEncoderID,
      Constants.SwerveDrivetrain.m_frontRightEncoderOffset,
      modulePositions[0]);

    modules[1] = new SwerveModule(
      "FrontLeft",
      Constants.SwerveDrivetrain.m_frontLeftDriveID,
      Constants.SwerveDrivetrain.m_frontLeftTurnID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderID,
      Constants.SwerveDrivetrain.m_frontLeftEncoderOffset,
      modulePositions[1]);

    modules[2] = new SwerveModule(
      "BackLeft",
      Constants.SwerveDrivetrain.m_backLeftDriveID,
      Constants.SwerveDrivetrain.m_backLeftTurnID,
      Constants.SwerveDrivetrain.m_backLeftEncoderID,
      Constants.SwerveDrivetrain.m_backLeftEncoderOffset,
      modulePositions[2]);

    modules[3] = new SwerveModule(
      "BackRight",
      Constants.SwerveDrivetrain.m_backRightDriveID,
      Constants.SwerveDrivetrain.m_backRightTurnID,
      Constants.SwerveDrivetrain.m_backRightEncoderID,
      Constants.SwerveDrivetrain.m_backRightEncoderOffset,
      modulePositions[3]);

    //Set PID controllers
    xController = new PIDController(Constants.SwerveDrivetrain.m_x_control_P, Constants.SwerveDrivetrain.m_x_control_I, Constants.SwerveDrivetrain.m_x_control_D);
    yController = new PIDController(Constants.SwerveDrivetrain.m_y_control_P, Constants.SwerveDrivetrain.m_y_control_I, Constants.SwerveDrivetrain.m_y_control_D);
    
    thetaController = new ProfiledPIDController(
      Constants.SwerveDrivetrain.m_r_control_P,
      Constants.SwerveDrivetrain.m_r_control_I,
      Constants.SwerveDrivetrain.m_r_control_D,
      Constants.SwerveDrivetrain.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    rotationControllerFeedBack = new PIDController(
      Constants.SwerveDrivetrain.m_r_control_P,
      Constants.SwerveDrivetrain.m_r_control_I,
      Constants.SwerveDrivetrain.m_r_control_D);

    trajectoryConfig = new TrajectoryConfig(
      Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
      Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration);
    trajectoryConfig.setKinematics(swerveKinematics);

    field = new Field2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /**
   * Zero the gyro
   */
  public void zeroGyro() {
    gyro.reset();
  }

  public AHRS getGyro(){
    return gyro;
  }

  /**
   * Get heading of gyro
   * use IEEEremainder because it uses formula:
   * dividend - (divisor x Math.Round(dividend / divisor))
   * versus the remainder operator (%) which uses:
   * (Math.Abs(dividend) - (Math.Abs(divisor) x (Math.Floor(Math.Abs(dividend) / Math.Abs(divisor))))) x Math.Sign(dividend)
   * 
   * @return heading angle in degrees
   */
  public double getHeading() {
    return -Math.IEEEremainder(gyro.getYaw() - gyroOffset_deg, 360.0);
  }
}
