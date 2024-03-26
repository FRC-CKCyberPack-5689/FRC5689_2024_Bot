// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RMap;

public class pickupNote_Command extends Command {
  private boolean cmd_finished;
  
  public pickupNote_Command() {
    addRequirements(RMap.m_intake_subsystem);

    RMap.controller.x().onFalse(new InstantCommand(() -> this.cmd_finished = true));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RMap.m_intake_subsystem.getSensor()) {
      RMap.m_intake_subsystem.stopMotor();
      cmd_finished = true;
    } else {
      RMap.m_intake_subsystem.setMotor(RMap.intakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_intake_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
