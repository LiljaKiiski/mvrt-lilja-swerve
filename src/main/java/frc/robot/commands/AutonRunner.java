// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
public class AutonRunner extends SequentialCommandGroup {

  private SwerveDrivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private PathConstraints constraints = new PathConstraints(
    Constants.SwerveDrivetrain.kMaxAutonDriveSpeed, 
    Constants.SwerveDrivetrain.kMaxAutonDriveAcceleration);

  /** Creates a new AutonRunner. */
  public AutonRunner(SwerveDrivetrain _drivetrain, String pathName) {
    this.drivetrain = _drivetrain;
    addRequirements(drivetrain);

    trajectory = PathPlanner.loadPath(
      pathName, 
      constraints);

    drivetrain.getField().getObject("traj").setTrajectory(trajectory);

    //Add events to hashmap, not used rn
    HashMap<String, Command> eventMap = new HashMap<>();

    List<PathPlannerTrajectory> fullTrajectoriesWithStopEvents = PathPlanner.loadPathGroup(
      pathName, 
      constraints);

    Command autoEventsCommand = drivetrain.getAutonBuilder(eventMap).fullAuto(fullTrajectoriesWithStopEvents);

    addCommands(
      new InstantCommand(() -> drivetrain.resetModules()),
      new InstantCommand(() -> drivetrain.resetModuleDrive()),
      new InstantCommand(() -> drivetrain.setModes(NeutralMode.Brake)),
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialHolonomicPose())),
      autoEventsCommand,
      new InstantCommand(() -> drivetrain.stopModules()));
  }
}