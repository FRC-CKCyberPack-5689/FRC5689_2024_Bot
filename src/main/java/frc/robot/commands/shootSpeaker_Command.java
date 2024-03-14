// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake_subsystem;
import frc.robot.subsystems.shooter_subsystem;

public class shootSpeaker_Command extends Command {
  private shooter_subsystem m_shooter_subsystem;
  private intake_subsystem m_intake_subsystem;
  private CommandXboxController controller;
  private boolean cmd_finished;
  
  public shootSpeaker_Command(shooter_subsystem shooter_subsystem, intake_subsystem intake_subsystem, CommandXboxController controller) {
    this.m_shooter_subsystem = shooter_subsystem;
    this.m_intake_subsystem = intake_subsystem;
    this.controller = controller;

    addRequirements(this.m_shooter_subsystem);
    addRequirements(this.m_intake_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter_subsystem.stopMotors();
    m_intake_subsystem.stopMotor();
    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter_subsystem.setMotorsSpeed(1);

    Timer.delay(0.25);

    m_intake_subsystem.setMotor(1);

    if (!controller.a().getAsBoolean()) {
      cmd_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter_subsystem.setMotorsSpeed(0);
    m_intake_subsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
