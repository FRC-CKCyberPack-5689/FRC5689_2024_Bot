// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RMap;

public class AutoPickupNoteDrive extends Command {
  PhotonTrackedTarget target;
  boolean cmd_finished, target_acquired;
  PIDController pidController;
  Supplier<Double> axis;

  public AutoPickupNoteDrive() {
    addRequirements(RMap.m_driveTrain_subsystem, RMap.m_intake_subsystem);

    pidController = new PIDController(0.0115, 0, 0.0001);
    axis = () -> RMap.controller.getRightTriggerAxis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd_finished = false;
    target_acquired = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult x = RMap.noteCamera.getLatestResult();
    double speedF = MathUtil.applyDeadband(axis.get(), 0.5) * 0.25;

    if (x.hasTargets()) {
      target = x.getBestTarget();
    } else {
      target = null;
    }

    if (target != null) {
      double error = pidController.calculate(target.getYaw(), 0.0);
      error = MathUtil.clamp(error, -1, 1.0);

      RMap.m_driveTrain_subsystem.drive(speedF, -error, 0);

      target_acquired = true;
    }
    
    if (target == null && target_acquired) {
      if (!RMap.m_intake_subsystem.getSensor()) {
        RMap.m_intake_subsystem.setMotor(RMap.intakeSpeed);
        RMap.m_driveTrain_subsystem.drive(speedF, 0, 0);
      } else {
        cmd_finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RMap.m_driveTrain_subsystem.stopMotors();
    RMap.m_intake_subsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd_finished;
  }
}
