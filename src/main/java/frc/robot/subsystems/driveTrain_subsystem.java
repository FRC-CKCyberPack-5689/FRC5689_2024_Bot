// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;

public class driveTrain_subsystem extends SubsystemBase {
  private CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  private MecanumDrive mecanumDrive;
  private ADIS16470_IMU gyro;

  public driveTrain_subsystem() {
    frontLeftMotor = new CANSparkMax(RMap.frontLeftMotor, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(RMap.frontRightMotor, MotorType.kBrushless);
    rearLeftMotor = new CANSparkMax(RMap.rearLeftMotor, MotorType.kBrushless);
    rearRightMotor = new CANSparkMax(RMap.rearRightMotor, MotorType.kBrushless);

    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    rearLeftMotor.setIdleMode(IdleMode.kBrake);
    rearRightMotor.setIdleMode(IdleMode.kBrake);

    frontRightMotor.setInverted(true);
    rearRightMotor.setInverted(true);

    mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    mecanumDrive.setDeadband(0);

    gyro = new ADIS16470_IMU();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
  }

  public void drive(double f, double r, double s) {
    mecanumDrive.driveCartesian(f, s, r);
  }

  public void driveGyro(double f, double r, double s, Rotation2d angle) {
    mecanumDrive.driveCartesian(f, s, r, angle);
  }

  public void stopMotors() {
    mecanumDrive.stopMotor();
  }

  public void feedWatchDogTimer() {
    mecanumDrive.feed();
  }

  public ADIS16470_IMU getGyro() {
    return gyro;
  }
}
