// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class SwerveModule {
  //Motors
  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  //Name
  private final String name;

  //Will assume for now these are same for each module
  private final boolean driveReversed = false;
  private final boolean turnReversed = true;
  private final boolean encoderReversed = false;

   //Encoder (CANCoder) - Magnetic rotary encoder
  private final double encoderOffsetRad;
  private final CANCoder encoder;

  //State TO reach
  private SwerveModuleState desiredState;

  //Current position
  private SwerveModulePosition position;

  /** Creates a new SwerveModule. */
  public SwerveModule(String name, int driveID, int turnID, int encoderID, 
                      double encoderOffsetRad, SwerveModulePosition position) {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed, Constants.SwerveDrivetrain.canivore_name);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed, Constants.SwerveDrivetrain.canivore_name);
  
    //Set PID constants
    driveMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kP);
    driveMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kI);
    driveMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kD);
    driveMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFF);

    turnMotor.config_kP(Constants.Talon.kPIDIdx, Constants.SwerveModule.kPTurn);
    turnMotor.config_kI(Constants.Talon.kPIDIdx, Constants.SwerveModule.kITurn);
    turnMotor.config_kD(Constants.Talon.kPIDIdx, Constants.SwerveModule.kDTurn);
    turnMotor.config_kF(Constants.Talon.kPIDIdx, Constants.SwerveModule.kFTurn);

    //Set turn to neutral (rotates freely)
    turnMotor.setNeutralMode(NeutralMode.Coast);

    //Set encoder
    this.encoderOffsetRad = encoderOffsetRad;
    encoder = new CANCoder(encoderID, Constants.SwerveDrivetrain.canivore_name);

    desiredState = new SwerveModuleState();
    this.name = "SwerveModule/" + name;

    resetEncoders();
    this.position = position;
  }

  /**
   * Reset encoders
   * Calibrate turn motor using encoder value
   */
  public void resetEncoders() {
   turnMotor.setSelectedSensorPosition(MathUtils.radiansToTicks(
        getEncoderRad(), 
        Constants.Talon.talonFXTicks, 
        Constants.SwerveModule.gear_ratio_turn)); 
  }

  /**
   * Get the angle of the absolute encoder sensor on the module
   * 
   * @return the angle of the swerve module 
   * (0 means forward with all screws facing left) CCW is positive
   */
  public double getEncoderRad() {
    double angle = encoder.getAbsolutePosition();
    angle = Math.toRadians(angle);
    angle -= encoderOffsetRad;
    return angle * (encoderReversed ? -1.0 : 1.0);
  }
}