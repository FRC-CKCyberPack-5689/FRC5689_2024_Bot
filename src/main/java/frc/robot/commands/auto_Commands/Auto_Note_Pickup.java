// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;

public class Auto_Note_Pickup extends Command {
  boolean cmd_finished;

  public Auto_Note_Pickup() {
    addRequirements(RMap.m_intake_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RMap.m_intake_subsystem.getSensor()) {
      RMap.m_intake_subsystem.setMotor(RMap.intakeSpeed);
    } else {
      cmd_finished = true;
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
