// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RMap;

public class TrajectoryTest extends SequentialCommandGroup {
  /** Creates a new TrajectoryTest. */
  public TrajectoryTest() {
    TrajectoryConfig config =
      new TrajectoryConfig(RMap.kMaxMetersPerSecond, RMap.kMaxAngularRate)
      .setKinematics(RMap.kDriveKinematics);

    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d()),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d()),
        config);

    MecanumControllerCommand controllerCommand =
      new MecanumControllerCommand(
        testTrajectory,
        RMap.m_driveTrain_subsystem::getPose,
        RMap.kFeedForward,
        RMap.kDriveKinematics,
        new PIDController(RMap.kPXController, 0, 0),
        new PIDController(RMap.kPYController, 0, 0),
        new ProfiledPIDController(RMap.kPThetaController, 0, 0, RMap.kThetaControllerConstraints),
        RMap.kMaxMetersPerSecond,
        new PIDController(0.5, 0, 0),
        new PIDController(0.5, 0, 0),
        new PIDController(0.5, 0, 0),
        new PIDController(0.5, 0, 0),
        RMap.m_driveTrain_subsystem::getWheelSpeeds,
        RMap.m_driveTrain_subsystem::setMotorControllerVolts,
        RMap.m_driveTrain_subsystem);

    addCommands(
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.resetOdometry(testTrajectory.getInitialPose())),
      controllerCommand,
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.stopMotors())
    );
  }
}
