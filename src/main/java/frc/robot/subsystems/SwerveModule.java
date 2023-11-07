// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.TalonFactory;

public class SwerveModule {
  //Motors
  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  private TalonFXSimCollection driveMotorSim = null;
  private TalonFXSimCollection turnMotorSim = null;

  //Name
  private final String name;

  //Will assume for now these are same for each module
  private final boolean driveReversed = false;
  private final boolean turnReversed = true;
  private final boolean encoderReversed = false;

   //Encoder (CANCoder) - Magnetic rotary encoder
  private final double encoderOffsetRad;
  private final CANCoder encoder;

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

    this.name = "SwerveModule/" + name;

    resetEncoders();
    this.position = position;
  }

  /**
   * set the desired state of the module
   * performs optimization based on currents state and velocity
   * @param state
   */
  public void setDesiredState(SwerveModuleState state) {
    //Stop module if too small change
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      disableModule();
      return;
    }
    SwerveModuleState optimized_state = optimize(state);
    setRawState(optimized_state);
    updatePosition();
  }

  /**
   * sets the raw state of the module
   * @param state
   */
  public void setRawState(SwerveModuleState state) {
    setAngle(state.angle.getRadians());
    setVelocity(state.speedMetersPerSecond);
  }

  /**
   * set the angle of the turn motor
   * 
   * @param radians
   */
  private void setAngle(double radians) {
    /*
     * Reset integral accumulator
     * It is sometimes desirable to clear the internal state 
     * (most importantly, the integral accumulator) of a 
     * PIDController, as it may be no longer valid (e.g. when
     * the PIDController has been disabled and then re-enabled)
     */
    if (Math.abs(
        MathUtils.ticksToRadians(turnMotor.getSelectedSensorPosition(),
          Constants.Talon.talonFXTicks, 
          Constants.SwerveModule.gear_ratio_turn)
          - radians) < 3 * Math.PI / 180) {
        turnMotor.setIntegralAccumulator(0);
    }

    //Set turn motor to angle
    turnMotor.set(
        ControlMode.Position,
        MathUtils.radiansToTicks(
          radians,
          Constants.Talon.talonFXTicks,
          Constants.SwerveModule.gear_ratio_turn));
  }

  /**
   * Use method to get raw cancoder offset for module
   * Turn all wheels with screws pointing left before using
   * 
   * @return in degrees offset
   */

  public double getRawCanCoderOffset(){
    return encoder.getAbsolutePosition();
  }

   /**
   * get the swerve module name (useful for SmartDashboard)
   */
  public void log() {
    SmartDashboard.putNumber(name + "/OffsetCancoderDeg", getRawCanCoderOffset());
 }
  
  /**
   * set the velocity of the drive motor
   * 
   * @param v_mps
   */
  private void setVelocity(double v_mps) {
    driveMotor.set(
        ControlMode.Velocity,
        MathUtils.rpmToTicks(
            MathUtils.mpsToRPM(v_mps, Constants.SwerveModule.radius),
            Constants.SwerveModule.gear_ratio_drive));
  }

  /**
   * Update the module position for kinematics/odometry
   */
  public void updatePosition() {
    Rotation2d angle = new Rotation2d(getTurnPosition());
    double distance = getDrivePosition();
    SmartDashboard.putString("Module Positions Values " + encoder.getDeviceID(), angle + " rad, " + distance + " meters");
    position.angle = angle;
    position.distanceMeters = distance;
  }

  /**
   * Get the drive position of the module
   * (aka how far module has driven)
   * 
   * @return the position, Units: meters
   */
  public double getDrivePosition() {
    return MathUtils.ticksToMeter(
      driveMotor.getSelectedSensorPosition(),
      Constants.Talon.talonFXTicks,
      Constants.SwerveModule.gear_ratio_drive,
      Constants.SwerveModule.radius);
  }

  /**
   * Get the turn position for the module
   * (aka how much has turned)
   * 
   * This method is just for formality for now,
   * does exact same as getRawEncoderRad()
   * 
   * @return the position in radians
   */
  public double getTurnPosition() {
    return getRawEncoderRad();
  }

  /**
   * Reset encoders
   * Calibrate turn motor using encoder value
   */
  public void resetEncoders() {
   turnMotor.setSelectedSensorPosition(MathUtils.radiansToTicks(
        getAbsoluteEncoderRad(), 
        Constants.Talon.talonFXTicks, 
        Constants.SwerveModule.gear_ratio_turn)); 
  }

  public void resetDriveEncoders() {
    driveMotor.setSelectedSensorPosition(0);
  }

  /**
   * Set mode
   */
  public void setMode(NeutralMode mode){
    driveMotor.setNeutralMode(mode);
    turnMotor.setNeutralMode(mode);
  }

  /**
   * Get the angle of the absolute encoder sensor on the module
   * 
   * @return the angle of the swerve module 
   * (0 means forward with all screws facing left) CCW is positive
   */
  public double getAbsoluteEncoderRad() {
    double angle = encoder.getAbsolutePosition();
    angle = Math.toRadians(angle);
    angle -= encoderOffsetRad;
    return angle * (encoderReversed ? -1.0 : 1.0);
  }

  /**
   * Get the angle of the wheel based on the talonfx
   * 0 is an arbitary position
   * CCW is positive
   * @return angle in rad
   */
  public double getRawEncoderRad() {
    return MathUtils.ticksToRadians(
      turnMotor.getSelectedSensorPosition(),
        Constants.Talon.talonFXTicks,
        Constants.SwerveModule.gear_ratio_turn);
  }

  /**
   * Stop the module from running
   */
  public void disableModule() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(ControlMode.PercentOutput, 0);
  }
  
  public String getName(){
    return name;
  }

  /**
   * Find the optimal angle for the module to go to (prevents it from ever
   * rotating more than 90 degrees at a time)
   * Full credits to Aman / Aakash for this method
   * 
   * @param state
   * @return optimal state
   */
  public SwerveModuleState optimize(SwerveModuleState state) {
    double targetAngle = state.angle.getDegrees();

    targetAngle %= 360.0;
    if (targetAngle < 0) {
      targetAngle += 360;
    }

    double currentAngle = getRawEncoderRad() * 180.0 / Math.PI;
    double currentAngleNormalized = currentAngle % 360.0;
    double diff = targetAngle - currentAngleNormalized;
    double targetVelocity = state.speedMetersPerSecond;

    if (90.0 < Math.abs(diff) && Math.abs(diff) < 270.0) {
      double beta = 180.0 - Math.abs(diff);
      beta *= Math.signum(diff);
      targetAngle = currentAngle - beta;
      targetVelocity *= -1;
    } else if (Math.abs(diff) >= 270.0) {
      if (diff < 0)
        targetAngle = currentAngle + (360.0 + diff);
      else
        targetAngle = currentAngle - (360.0 - diff);
    } else {
      targetAngle = currentAngle + diff;
    }

    SwerveModuleState newState = new SwerveModuleState(targetVelocity, new Rotation2d(targetAngle * Math.PI / 180.0));
    return newState;
  }
}