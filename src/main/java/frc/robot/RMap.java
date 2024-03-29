// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.driveTrain_subsystem;
import frc.robot.subsystems.intake_subsystem;
import frc.robot.subsystems.lift_subsystem;
import frc.robot.subsystems.shooter_subsystem;

public class RMap {
    // CAN BUS
    public static final int frontLeftMotor = 2;
    public static final int frontRightMotor = 3;
    public static final int rearLeftMotor = 4;
    public static final int rearRightMotor = 5;
    public static final int leftShooterMotor = 7;
    public static final int rightShooterMotor = 6;
    public static final int rearLiftMotor = 8;
    public static final int frontLiftMotor = 9;
    public static final int intakeMotor = 10;

    // Input Devices
    public static final int driverControllerPort = 0;
    public static final int noteSensor = 0;

    //Tunables
    //Drivetrain
    public static final double driveTrainMaxSpeed = 1;
    public static final double driveTrainMaxAccel = 0.02;
    public static final double driveTrainMaxDecel = driveTrainMaxAccel * 2.0;

    //Intake & Amp Feed Speeds
    public static final double intakeSpeed = 0.50;
    public static final double ampSpeed = 0.15;

    //Lift System Parameters
    public static final int liftSystemCurrentLimit = 80;
    public static final double liftMaxHeight = 35;

    public static final double liftP = 0.09;
    public static final double liftI = 0.00003;
    public static final double liftD = 0.00;
    public static final double liftFF = 0.00;
    public static final double lift_PID_Output_Min = -0.5;
    public static final double lift_PID_Output_Max = 0.5;

    //Shooter Parameters
    public static final double shooterP = 10.00;
    public static final double shooterI = 0.05;
    public static final double shooterD = 0.00;
    public static final double shooterFF = 10.00;
    public static final double shooter_PID_Output_Min = -1.00;
    public static final double shooter_PID_Output_Max = 0;
    public static final double shooterNoteBumpAmt = 1.5;
    public static final double shooterDelay = 0.25;

    //Autonomous Drive Parameters
    public static final double autoDriveTime = 6;
    public static final double autoDriveSpeed = 0.1;
    public static final double autoShootAndDriveTime = 6;

    //Autonomous Shoot Parameters
    public static final double shooterTime = 1;

    //Build Attributes
    public static final double driveTrainGearRatio = 12.75;
    public static final double wheelDiameterMeters = Units.inchesToMeters(8);
    public static final double wheelCircMeters = (Math.PI * wheelDiameterMeters);
    public static final double wheelVelocityConversionFactor = (Math.PI * wheelDiameterMeters / 60.0);

    //Subsystems & Global Objects
    public static ADIS16470_IMU gyro;
    public static CommandXboxController controller;
    public static driveTrain_subsystem m_driveTrain_subsystem;
    public static shooter_subsystem m_shooter_subsystem;
    public static intake_subsystem m_intake_subsystem;
    public static lift_subsystem m_lift_subsystem;
    public static PhotonCamera noteCamera;

    public static final double kTrackWidth = Units.inchesToMeters(20.25);
    public static final double kTrackBase = Units.inchesToMeters(23.0);

    public static final double kMaxMetersPerSecond = 3.0;
    public static final double kMaxAccelerationSquared = 3.0;
    public static final double kMaxAngularRate = Math.PI;
    public static final double kMaxAngularAccelRate = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kTrackBase / 2, kTrackWidth / 2),
            new Translation2d(kTrackBase / 2, -kTrackWidth / 2),
            new Translation2d(-kTrackBase / 2, kTrackWidth / 2),
            new Translation2d(-kTrackBase / 2, -kTrackWidth / 2));

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(kMaxAngularRate, kMaxAngularAccelRate);

    public static final SimpleMotorFeedforward kFeedForward =
        new SimpleMotorFeedforward(1, 2.45, 0.43);
}
