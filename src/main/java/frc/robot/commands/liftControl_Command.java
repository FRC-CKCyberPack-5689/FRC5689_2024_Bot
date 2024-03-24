// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap;

public class liftControl_Command extends Command {
  private CommandXboxController controller;
  
  public liftControl_Command() {
    this.controller = RMap.controller;

    addRequirements(RMap.m_lift_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double i = controller.getLeftTriggerAxis();
    i = MathUtil.applyDeadband(i, 0.02);
    i *= RMap.liftMaxHeight;

    RMap.m_lift_subsystem.setPosition(i);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_lift_subsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
