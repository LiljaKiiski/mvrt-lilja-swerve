// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.Constants;
import frc.robot.utils.TalonFactory;

public class SwerveModule {

  private final BaseTalon driveMotor;
  private final BaseTalon turnMotor;

  private final boolean driveReversed = false;
  private final boolean turnReversed = true;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int turnID) {
    driveMotor = TalonFactory.createTalonFX(driveID, driveReversed, Constants.SwerveDrivetrain.canivore_name);
    turnMotor = TalonFactory.createTalonFX(turnID, turnReversed, Constants.SwerveDrivetrain.canivore_name);
  }
}
