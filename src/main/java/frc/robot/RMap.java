// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class RMap {
    // CAN BUS
    public static final int frontLeftMotor = 2;
    public static final int frontRightMotor = 3;
    public static final int rearLeftMotor = 4;
    public static final int rearRightMotor = 5;
    public static final int leftShooterMotor = 6;
    public static final int rightShooterMotor = 7;
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
    public static final double ampSpeed = 0.2;

    //Lift System Parameters
    public static final int liftSystemCurrentLimit = 80;
    public static final double liftMaxHeight = 38;

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
}
