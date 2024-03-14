// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap;
import frc.robot.subsystems.driveTrain_subsystem;

public class defaultStickDrive_Command extends Command {
  private driveTrain_subsystem m_driveTrain_subsystem;
  private CommandXboxController controller;
  private double speedF, speedR, speedS;

  public defaultStickDrive_Command(driveTrain_subsystem system, CommandXboxController controller) {
    this.m_driveTrain_subsystem = system;
    this.controller = controller;

    addRequirements(this.m_driveTrain_subsystem);
  }

  @Override
  public void initialize() {
    speedF = 0;
    speedR = 0;
    speedS = 0;
  }

  @Override
  public void execute() {
    speedF = calcuateDriveAxis(-controller.getLeftY(), speedF);
    speedS = calcuateDriveAxis(controller.getLeftX(), speedS);
    speedR = calcuateDriveAxis(controller.getRightX(), speedR);

    m_driveTrain_subsystem.drive(speedF, speedR, speedS);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; //This Command is the default and must never finish
  }

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
