// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.subsystems.driveTrain_subsystem;

public class Auto_Drive_Command extends Command {
  /** Creates a new Auto_Drive_Command. */
  private driveTrain_subsystem m_driveTrain;
  private boolean finished;

  public Auto_Drive_Command(driveTrain_subsystem driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_driveTrain = driveTrain;

    addRequirements(this.m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.drive(RMap.autoDriveSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
