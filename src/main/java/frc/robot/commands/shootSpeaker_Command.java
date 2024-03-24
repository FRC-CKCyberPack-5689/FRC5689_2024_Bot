// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;

public class shootSpeaker_Command extends Command {
  private boolean cmd_finished;
  private double initTimeStamp;
  
  public shootSpeaker_Command() {

    addRequirements(RMap.m_shooter_subsystem);
    addRequirements(RMap.m_intake_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RMap.m_shooter_subsystem.stopMotors();
    RMap.m_intake_subsystem.stopMotor();

    initTimeStamp = Timer.getFPGATimestamp();

    cmd_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RMap.m_shooter_subsystem.setMotorsSpeed(1);
    if ((Timer.getFPGATimestamp() - initTimeStamp) <= RMap.shooterTime) {
      // Waits a certain time before turning on intake to let shooter spin up first
      if ((Timer.getFPGATimestamp() - initTimeStamp) >= RMap.shooterDelay){
        RMap.m_intake_subsystem.setMotor(RMap.intakeSpeed);
      }
    } else {
      cmd_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_shooter_subsystem.setMotorsSpeed(0);
    RMap.m_intake_subsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
