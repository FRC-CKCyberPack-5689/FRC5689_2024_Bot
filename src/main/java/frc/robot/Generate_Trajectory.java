// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

/** Add your docs here. */
public class Generate_Trajectory {
    public static MecanumControllerCommand generateTrajectoryCMD(Trajectory trajectory) {
        MecanumControllerCommand controller =
            new MecanumControllerCommand(
                trajectory,
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
        
        return controller;
    }
}
