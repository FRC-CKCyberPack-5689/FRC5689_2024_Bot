// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap;
import frc.robot.subsystems.intake_subsystem;

public class pickupNote_Command extends Command {
  private intake_subsystem m_intake_subsystem;
  private CommandXboxController controller;
  private boolean cmd_finished;
  private DigitalInput note_Sensor;
  
  public pickupNote_Command() {
    this.m_intake_subsystem = RMap.m_intake_subsystem;
    this.controller = RMap.controller;
    this.note_Sensor = new DigitalInput(RMap.noteSensor);

    addRequirements(m_intake_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (note_Sensor.get()) {
      m_intake_subsystem.stopMotor();
      Timer.delay(0.25);
      cmd_finished = true;
    } else {
      m_intake_subsystem.setMotor(RMap.intakeSpeed);
    }

    if (!controller.x().getAsBoolean()) {
      cmd_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
