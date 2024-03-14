// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;

public class shooter_subsystem extends SubsystemBase {
  private CANSparkMax leftShooterMotor, rightShooterMotor;
  private SparkPIDController pid;
  private RelativeEncoder encoder;

  public shooter_subsystem() {
    leftShooterMotor = new CANSparkMax(RMap.leftShooterMotor, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(RMap.rightShooterMotor, MotorType.kBrushless);

    leftShooterMotor.setInverted(true);
    rightShooterMotor.follow(leftShooterMotor, true);

    pid = leftShooterMotor.getPIDController();
    pid.setP(RMap.shooterP);
    pid.setI(RMap.shooterI);
    pid.setD(RMap.shooterD);
    pid.setFF(RMap.shooterFF);
    pid.setOutputRange(RMap.shooter_PID_Output_Min, RMap.shooter_PID_Output_Max);

    encoder = leftShooterMotor.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorsSpeed(double i) {
    leftShooterMotor.set(i);
  }

  public void closedLoopControl(double target) {
    pid.setReference(target, ControlType.kPosition);
  }

  public void stopMotors() {
    leftShooterMotor.stopMotor();
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }
}
