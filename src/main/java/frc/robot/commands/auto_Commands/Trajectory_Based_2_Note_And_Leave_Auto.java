// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Generate_Trajectory;
import frc.robot.RMap;
import frc.robot.commands.shootSpeaker_Command;

public class Trajectory_Based_2_Note_And_Leave_Auto extends SequentialCommandGroup {

  TrajectoryConfig config =
    new TrajectoryConfig(1.0, 1)
      .setKinematics(RMap.kDriveKinematics);

  double x = Units.inchesToMeters(79.83);

  Trajectory leaveSubwooferTraj = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d()),
    List.of(new Translation2d(x / 2, 0)),
    new Pose2d(x, 0, new Rotation2d()),
    config);

  Trajectory gobackTraj = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d()),
    List.of(new Translation2d(-(x / 2), 0)),
    new Pose2d(-x, 0, new Rotation2d()),
    config.setReversed(true));

  MecanumControllerCommand fwdCMD = Generate_Trajectory.generateTrajectoryCMD(leaveSubwooferTraj);
  MecanumControllerCommand revCMD = Generate_Trajectory.generateTrajectoryCMD(gobackTraj);
  MecanumControllerCommand leaveCMD = Generate_Trajectory.generateTrajectoryCMD(leaveSubwooferTraj);

  public Trajectory_Based_2_Note_And_Leave_Auto() {
    addCommands(
      new shootSpeaker_Command(),
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.resetOdometry(leaveSubwooferTraj.getInitialPose())),
      new ParallelCommandGroup(
        new Auto_Note_Pickup(),
        fwdCMD
      ),
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.resetOdometry(gobackTraj.getInitialPose())),
      revCMD,
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.stopMotors()),
      new shootSpeaker_Command(),
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.resetOdometry(leaveSubwooferTraj.getInitialPose())),
      leaveCMD,
      new InstantCommand(() -> RMap.m_driveTrain_subsystem.stopMotors())
    );
  }
}
