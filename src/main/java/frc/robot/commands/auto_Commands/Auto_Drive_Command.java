// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;

public class Auto_Drive_Command extends Command {
  /** Creates a new Auto_Drive_Command. */
  private boolean finished;

  public Auto_Drive_Command() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RMap.m_driveTrain_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RMap.m_driveTrain_subsystem.drive(RMap.autoDriveSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_driveTrain_subsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
