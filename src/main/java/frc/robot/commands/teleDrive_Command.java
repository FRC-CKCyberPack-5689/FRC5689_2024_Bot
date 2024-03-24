// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RMap;

public class teleDrive_Command extends Command {
  private double speedF, speedR, speedS;
  // private double desiredPosition;

  public teleDrive_Command() {
    addRequirements(RMap.m_driveTrain_subsystem);
  }

  @Override
  public void initialize() {
    speedF = 0;
    speedR = 0;
    speedS = 0;

    // desiredPosition = 0;
  }

  @Override
  public void execute() {
    speedF = calcuateDriveAxis(-RMap.controller.getLeftY(), speedF);
    speedS = calcuateDriveAxis(RMap.controller.getLeftX(), speedS);
    speedR = calcuateDriveAxis(RMap.controller.getRightX(), speedR);

    RMap.controller.rightBumper().onTrue(new InstantCommand(() -> RMap.m_driveTrain_subsystem.drive(speedF, speedR, speedS)));
    RMap.controller.rightBumper().onFalse(new InstantCommand(
      () -> RMap.m_driveTrain_subsystem.driveGyro(speedF, speedR, speedS, Rotation2d.fromDegrees(-RMap.gyro.getAngle()))));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; //This command is the default and must never finish
  }

  /**
   * @param input Controller stick input
   * @param currentSpeed Current axis speed of robot
   * @return Ramped and cliped motor speed 
   */
  private double calcuateDriveAxis(double input, double currentSpeed) {
    double i = Math.abs(input);
    double c = currentSpeed;

    i = MathUtil.applyDeadband(i, 0.05);
    c = Math.abs(c);

    if (i > c) {
      i = Math.min(i, c + RMap.driveTrainMaxAccel);
    } else {
      i = Math.max(i, c - RMap.driveTrainMaxDecel);
    }

    i *= RMap.driveTrainMaxSpeed;

    i = Math.copySign(i, input);
    return i;
  }
}
