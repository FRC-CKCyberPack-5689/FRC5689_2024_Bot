// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;

public class bumpNote_Command extends Command {
  private boolean cmd_finished;

  public bumpNote_Command() {

    addRequirements(RMap.m_shooter_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RMap.m_shooter_subsystem.resetEncoder();
    RMap.m_shooter_subsystem.closedLoopControl(-RMap.shooterNoteBumpAmt);
    Timer.delay(0.5);
    RMap.m_shooter_subsystem.stopMotors();
    cmd_finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_shooter_subsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
