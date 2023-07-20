// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
public class AutonRunner extends CommandBase {

  private SwerveDrivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private PathConstraints constraints = new PathConstraints(
    Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
    Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration);

  /** Creates a new AutonRunner. */
  public AutonRunner(SwerveDrivetrain drivetrain, String pathName) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    trajectory = PathPlanner.loadPath(pathName, constraints);

    drivetrain.getField().getObject("traj").setTrajectory(trajectory);

    //Add events to hashmap, not used rn
    HashMap<String, Command> eventMap = new HashMap<>();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
