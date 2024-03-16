// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;

public class lift_subsystem extends SubsystemBase {
  private CANSparkMax frontLiftMotor, rearLiftMotor;
  private RelativeEncoder encoder;
  private SparkPIDController pid;

  public lift_subsystem() {
    frontLiftMotor = new CANSparkMax(RMap.frontLiftMotor, MotorType.kBrushless);
    rearLiftMotor = new CANSparkMax(RMap.rearLiftMotor, MotorType.kBrushless);

    rearLiftMotor.follow(frontLiftMotor);

    frontLiftMotor.setSmartCurrentLimit(RMap.liftSystemCurrentLimit);
    rearLiftMotor.setSmartCurrentLimit(RMap.liftSystemCurrentLimit);

    frontLiftMotor.setIdleMode(IdleMode.kBrake);
    rearLiftMotor.setIdleMode(IdleMode.kBrake);

    encoder = frontLiftMotor.getEncoder();
    encoder.setPosition(0);

    pid = frontLiftMotor.getPIDController();
    pid.setP(RMap.liftP);
    pid.setI(RMap.liftI);
    pid.setD(RMap.liftD);
    pid.setFF(RMap.liftFF);
    pid.setOutputRange(RMap.lift_PID_Output_Min, RMap.lift_PID_Output_Max);
    
  }

  @Override
  public void periodic() {}

  public void driveMotors(double i) {
    frontLiftMotor.set(i);
  }

  public void stopMotors() {
    frontLiftMotor.stopMotor();
  }

  public void setPosition(double i) {
    pid.setReference(i, CANSparkBase.ControlType.kPosition);
  }
}
