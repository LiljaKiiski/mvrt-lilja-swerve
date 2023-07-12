// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.JoystickIO;

public class SwerveJoystickCommand extends CommandBase {
  //Pointer to drivetrain
  private final SwerveDrivetrain drivetrain;

  //Joystick
  private final JoystickIO joystick;

  //Suppliers to functions
  private final Supplier<Double> xSpeedFunc;
  private final Supplier<Double> ySpeedFunc;
  private final Supplier<Double> turnSpeedFunc;

  //Trigger to change orientation
  private final Trigger fieldOrientedFunc;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(
      SwerveDrivetrain drivetrain, 
      Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> angularSpeedFunc, 
      Trigger fieldOrientedFunc, JoystickIO joystick) {
    
    //Set everything up
    this.drivetrain = drivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turnSpeedFunc = angularSpeedFunc;
    this.fieldOrientedFunc = fieldOrientedFunc;

    this.joystick = joystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
