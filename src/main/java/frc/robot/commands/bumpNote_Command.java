// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;
import frc.robot.subsystems.shooter_subsystem;

public class bumpNote_Command extends Command {
  private shooter_subsystem m_shooter;
  private boolean cmd_finished;

  public bumpNote_Command(shooter_subsystem shooter) {
    this.m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.resetEncoder();
    m_shooter.closedLoopControl(-RMap.shooterNoteBumpAmt);
    Timer.delay(0.5);
    m_shooter.stopMotors();
    cmd_finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
