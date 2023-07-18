// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class JoystickControls {
    public static final boolean xBoxControl = false;
    public static final boolean invertJoystickX = true;
    public static final boolean invertJoystickY = true;
    public static final boolean invertJoystickW = true;
    
    public static final double kPJoystick = 0.001;
    public static final double kIJoystick = 0.0;
    public static final double kDJoystick = 0.0;
    public static final double kFJoystick = 0.0;

}
  public static class SwerveDrivetrain {
    // Physical Constants
    public static final double chassisWidth = Units.inchesToMeters(26);
    public static final double chassisLength = Units.inchesToMeters(28);// swap for comp

    // Important locations for swerve
    // consider swapping corners
    public static final Translation2d m_standardCenterLocation = new Translation2d(0, 0);
    public static final Translation2d m_frontLeftLocation = new Translation2d(chassisLength / 2.0,
                    chassisWidth / 2.0);
    public static final Translation2d m_frontRightLocation = new Translation2d(chassisLength / 2.0,
                    -chassisWidth / 2.0);
    public static final Translation2d m_backLeftLocation = new Translation2d(-chassisLength / 2.0,
                    chassisWidth / 2.0);
    public static final Translation2d m_backRightLocation = new Translation2d(-chassisLength / 2.0,
                    -chassisWidth / 2.0);
    public static final Translation2d[] rotatePoints = {
                    m_standardCenterLocation,
                    m_frontLeftLocation,
                    m_frontRightLocation,
                    m_backLeftLocation,
                    m_backRightLocation
    };

    // Motor ID
    public static final int m_frontRightDriveID = 1;
    public static final int m_frontLeftDriveID = 3;
    public static final int m_backLeftDriveID = 5;
    public static final int m_backRightDriveID = 7;

    public static final int m_frontRightTurnID = 2;
    public static final int m_frontLeftTurnID = 4;
    public static final int m_backLeftTurnID = 6;
    public static final int m_backRightTurnID = 8;

    // Abs Encoder ID
    public static final int m_frontRightEncoderID = 9;
    public static final int m_frontLeftEncoderID = 10;
    public static final int m_backLeftEncoderID = 11;
    public static final int m_backRightEncoderID = 12;

    // Comp Bot Encoder Offsets
    public static final boolean isCompBot = true;
    public static final String canivore_name = (isCompBot)? "Drivetrain" : "rio";

    public static final double m_frontLeftEncoderOffset_Comp = Units.degreesToRadians(221.04); // 128.32// + Math.PI/2.0;
    public static final double m_frontRightEncoderOffset_Comp = Units.degreesToRadians(273.42); // 115.04// + Math.PI/2.0;
    public static final double m_backLeftEncoderOffset_Comp = Units.degreesToRadians(213.398); //214.8 // + Math.PI/2.0;
    public static final double m_backRightEncoderOffset_Comp = Units.degreesToRadians(79.18); // 341.81// + Math.PI/2.0;

    // Practice Bot Encoder Offsets
    public static final double m_frontLeftEncoderOffset_P = Units.degreesToRadians(33.62);// + Math.PI/2.0;
    public static final double m_frontRightEncoderOffset_P = Units.degreesToRadians(182.180);// + Math.PI/2.0;
    public static final double m_backLeftEncoderOffset_P = Units.degreesToRadians(341.54);// + Math.PI/2.0;
    public static final double m_backRightEncoderOffset_P = Units.degreesToRadians(146.86);// + Math.PI/2.0;

    // Abs Encoder Offsets
    public static final double m_frontLeftEncoderOffset = isCompBot ? m_frontLeftEncoderOffset_Comp
                    : m_frontLeftEncoderOffset_P;// + Math.PI/2.0;
    public static final double m_frontRightEncoderOffset = isCompBot ? m_frontRightEncoderOffset_Comp
                    : m_frontRightEncoderOffset_P;// + Math.PI/2.0;
    public static final double m_backLeftEncoderOffset = isCompBot ? m_backLeftEncoderOffset_Comp
                    : m_backLeftEncoderOffset_P;// + Math.PI/2.0;
    public static final double m_backRightEncoderOffset = isCompBot ? m_backRightEncoderOffset_Comp
                    : m_backRightEncoderOffset_P;// + Math.PI/2.0;

    // constants for joystick drive
    public static final double kSensitivity = 0.90;
    public static final double kWheelDeadband = 0.2;
    public static final double kThrottleDeadband = 0.2;
    public static final double kWheelGain = 0.05;
    public static final double kWheelNonlinearity = 0.05;

    public static final double kMaxSpeedMPS = 10; // optimize max speed to prioritize translation
    public static final double kDriveMaxAccelerationNormal = 3;
    public static final double kTurnMaxAccelerationNormal = 0.5 * Math.PI;
    public static final double kDriveMaxSpeedMPSNormal = 3.5;
    public static final double kTurnMaxSpeedRPSNormal = 1 * Math.PI;
    public static final double kDriveMaxSpeedCap = 10;
    public static double kDriveMaxSpeedMPS = kDriveMaxSpeedMPSNormal;
    public static double kTurnMaxSpeedRPS = kTurnMaxSpeedRPSNormal;
    public static double kDriveMaxAcceleration = kDriveMaxAccelerationNormal;
    public static double kTurnMaxAcceleration = kTurnMaxAccelerationNormal;
    
    public static final int kDriveJoystickPort = 0;
    public static final int kDriveXAxis = 0;
    public static final int kDriveYAxis = 1;
    public static final int kDriveWAxis = 4;
    public static final int kDriveFieldOrientButtonIdx = 8;
    public static final int kDriveLeftTrigger = 2;
    public static final int kDriveRightTrigger = 3;

    // values to be determined after the robot is characterized
    public static final double kS = 0; // 0.69382 //units: Volts
    public static final double kV = 0; // 1.30485 //2.6097 //units: Volts * Seconds / Meters
    public static final double kA = 0; // 0.35228 //units: Volts * Seconds^2 / Meters

    // Position PID
    public static final double m_x_control_P = 1.6;
    public static final double m_x_control_I = 0.5;
    public static final double m_x_control_D = 0.0;
    public static final double m_y_control_P = 1.6;
    public static final double m_y_control_I = 0.5;
    public static final double m_y_control_D = 0.0;
    public static final double m_r_control_P = 2.0;
    public static final double m_r_control_I = 0.0;
    public static final double m_r_control_D = 0;

    // Auton Constants
    public static final double kMaxAutonDriveSpeed = 4; // mps
    public static final double kMaxAutonDriveAcceleration = 3; // mps2
    public static final double kMaxAutonThetaVelocity = kMaxAutonDriveSpeed
                    / Math.hypot(chassisWidth / 2.0, chassisLength / 2.0); // rad ps
    public static final double kMaxAutonThetaAcceleration = kMaxAutonDriveAcceleration
                    / Math.hypot(chassisWidth / 2.0, chassisLength / 2.0); // rad ps^2

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                    kMaxAutonThetaVelocity, kMaxAutonThetaAcceleration);

    public static final double kTeleopHeadingCorrectionScale = 0;
  }
  
  public static class SwerveModule {
    //Wheel / gear ratios
    public static final double gear_ratio_turn = 150.0 / 7.0; // number of rotations of talon for one turn of wheel
    public static final double gear_ratio_drive = 6.75 / 1.0; // number of rotations of talon for one rotation of wheel
    public static final double radius = 0.05; // Units: m
    public static final double kwheelCircumference = 2 * Math.PI * radius; // Units: m

    // PID Constants
    public static final double kP = 0.1;;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;

    public static final double kPTurn = 0.3;
    public static final double kITurn = 0.0;
    public static final double kDTurn = 0.0;
    public static final double kFTurn = 0.0;
}

  public static class Talon {
    //Ticks
    public static final int talonFXTicks = 2048;
    public static final int talonSRXTicks = 4096;

    //Other
    public static final double MAX_VOLTAGE = 10.0;
    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final double kVoltage = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(
                    true,
                    35,
                    50, 0.75);

  }
}
