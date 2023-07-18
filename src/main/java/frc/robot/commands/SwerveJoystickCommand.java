// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.JoystickIO;
import frc.robot.utils.MathUtils;

public class SwerveJoystickCommand extends CommandBase {
  //Pointer to drivetrain
  private final SwerveDrivetrain drivetrain;

  //Joystick
  private final JoystickIO joystick;

  //Suppliers to functions
  private final Supplier<Double> xSpeedFunc;
  private final Supplier<Double> ySpeedFunc;
  private final Supplier<Double> turnSpeedFunc;

  //Current heading (as a Rotation2d)
  private Rotation2d heading;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(
      SwerveDrivetrain drivetrain, 
      Supplier<Double> xSpeedFunc, Supplier<Double> ySpeedFunc, Supplier<Double> angularSpeedFunc, 
      JoystickIO joystick) {  
  
    //Set everything up
    this.drivetrain = drivetrain;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.turnSpeedFunc = angularSpeedFunc;

    this.joystick = joystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Get values from joystick
    double vX = xSpeedFunc.get(); // as of here, negative X is backwards, positive X is forward
    double vY = ySpeedFunc.get(); // as of here, positive Y is left, negative Y is right
    double vW = turnSpeedFunc.get(); // as of here, negative W is down (CW) positive W is up (CCW)

    /*
     * Deadbands are a common technique to sanitized joystick input
     * and eliminate erroneous input
     */
    // apply deadband
    vX = MathUtils.handleDeadband(vX, Constants.SwerveDrivetrain.kThrottleDeadband);
    vY = MathUtils.handleDeadband(vY, Constants.SwerveDrivetrain.kThrottleDeadband);
    vW = MathUtils.handleDeadband(vW, Constants.SwerveDrivetrain.kWheelDeadband);

    // limit acceleration
    vX *= Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vY *=  Constants.SwerveDrivetrain.kDriveMaxSpeedMPS;
    vW *= Constants.SwerveDrivetrain.kTurnMaxSpeedRPS;

    //Logging
    SmartDashboard.putBoolean("Field Oriented", drivetrain.isFieldOriented());

    //Heading correction
    if (MathUtils.withinEpsilon(vW, 0, 0.01)) {
      double v_w_compensate = drivetrain.holdHeading(heading);
      vW += v_w_compensate;
      SmartDashboard.putBoolean("Holding Heading", true);
    }
    else {
      heading = drivetrain.getRotation2d();
      SmartDashboard.putBoolean("Holding Heading", false);
    }

    //Update drivetrain speeds
    drivetrain.setSpeeds(vX, vY, vW);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
