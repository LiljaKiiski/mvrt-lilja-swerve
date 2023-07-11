// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

public class SwerveModule {

  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  /** Creates a new SwerveModule. */
  public SwerveModule() {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed, Constants.SwerveDrivetrain.canivore_name);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed, Constants.SwerveDrivetrain.canivore_name);
  }
}
