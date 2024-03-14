// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;

public class intake_subsystem extends SubsystemBase {
  private CANSparkMax intake_motor;

  public intake_subsystem() {
    intake_motor = new CANSparkMax(RMap.intakeMotor, MotorType.kBrushless);
    intake_motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {}

  public void setMotor(double setpoint) {
    intake_motor.set(setpoint);
  }
  
  public void stopMotor() {
    intake_motor.stopMotor();
  }
}
