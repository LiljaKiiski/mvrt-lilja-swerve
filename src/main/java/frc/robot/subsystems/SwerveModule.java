// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

public class SwerveModule {

  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;
  private final String name;

  //Will assume for now these are same for each module
  private final boolean driveReversed = false;
  private final boolean turnReversed = true;
  private final boolean encoderReversed = false;

  /** Creates a new SwerveModule. */
  public SwerveModule(String name, int driveID, int turnID, int encoderID) {
    this.name = name;
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

    turnMotor.setNeutralMode(NeutralMode.Coast);
  }
}