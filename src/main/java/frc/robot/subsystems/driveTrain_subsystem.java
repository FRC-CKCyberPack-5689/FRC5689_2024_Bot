// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RMap;

public class driveTrain_subsystem extends SubsystemBase {
  private CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
  private RelativeEncoder fL_encoder, fR_encoder, rL_encoder, rR_encoder;
  private MecanumDrive mecanumDrive;
  private MecanumDriveOdometry odometry;

  public driveTrain_subsystem() {
    RMap.gyro = new ADIS16470_IMU();
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

    fL_encoder = frontLeftMotor.getEncoder();
    fR_encoder = frontRightMotor.getEncoder();
    rL_encoder = rearLeftMotor.getEncoder();
    rR_encoder = rearRightMotor.getEncoder();

    fL_encoder.setPositionConversionFactor(RMap.wheelCirc / RMap.driveTrainGearRatio);
    fR_encoder.setPositionConversionFactor(RMap.wheelCirc / RMap.driveTrainGearRatio);
    rL_encoder.setPositionConversionFactor(RMap.wheelCirc / RMap.driveTrainGearRatio);
    rR_encoder.setPositionConversionFactor(RMap.wheelCirc / RMap.driveTrainGearRatio);

    resetEncoders();

    odometry = new MecanumDriveOdometry(
      RMap.kDriveKinematics,
      new Rotation2d(getHeading()),
      new MecanumDriveWheelPositions());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Angle", getHeading());

    odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelPosition());
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

  public void resetEncoders() {
    fL_encoder.setPosition(0);
    fR_encoder.setPosition(0);
    rL_encoder.setPosition(0);
    rR_encoder.setPosition(0);
  }

  public void setMotorControllerVolts(MecanumDriveMotorVoltages motorVolts) {
    frontLeftMotor.setVoltage(motorVolts.frontLeftVoltage);
    frontRightMotor.setVoltage(motorVolts.frontRightVoltage);
    rearLeftMotor.setVoltage(motorVolts.rearLeftVoltage);
    rearRightMotor.setVoltage(motorVolts.rearRightVoltage);
  }

  public void setMaximumOutput(double maxSpeed) {
    mecanumDrive.setMaxOutput(maxSpeed);
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      fL_encoder.getVelocity(),
      fR_encoder.getVelocity(),
      rL_encoder.getVelocity(),
      rR_encoder.getVelocity());
  }

  public MecanumDriveWheelPositions getWheelPosition() {
    return new MecanumDriveWheelPositions(
      fL_encoder.getPosition(),
      fR_encoder.getPosition(),
      rL_encoder.getPosition(),
      rR_encoder.getPosition());
  }

  public void zeroHeading() {
    RMap.gyro.reset();
  }

  public double getHeading() {
    return -RMap.gyro.getAngle();
  }

  public double getTurnRate() {
    return -RMap.gyro.getRate();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getWheelPosition(), pose);
  }
}
