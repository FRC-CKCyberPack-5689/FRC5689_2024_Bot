// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Implement Auto Shoot, Move Back, Pickup Note, Reverse, Auto Shoot, Leave

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.bumpNote_Command;
import frc.robot.commands.teleDrive_Command;
import frc.robot.commands.liftControl_Command;
import frc.robot.commands.pickupNote_Command;
import frc.robot.commands.shootAmp_Command;
import frc.robot.commands.shootSpeaker_Command;
import frc.robot.commands.auto_Commands.Auto_Drive_Command;
import frc.robot.commands.auto_Commands.Auto_Shoot_Command;
import frc.robot.subsystems.driveTrain_subsystem;
import frc.robot.subsystems.intake_subsystem;
import frc.robot.subsystems.lift_subsystem;
import frc.robot.subsystems.shooter_subsystem;

public class RobotContainer {
  private SendableChooser<Command> chooser;
  private Auto_Drive_Command auto_Drive_Command;
  private Auto_Shoot_Command auto_Shoot_Command;
  private shootSpeaker_Command shootSpeaker_Command;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RMap.controller = new CommandXboxController(RMap.driverControllerPort);
    RMap.m_driveTrain_subsystem = new driveTrain_subsystem();
    RMap.m_shooter_subsystem = new shooter_subsystem();
    RMap.m_intake_subsystem = new intake_subsystem();
    RMap.m_lift_subsystem = new lift_subsystem();

    auto_Drive_Command = new Auto_Drive_Command();
    auto_Shoot_Command = new Auto_Shoot_Command();
    shootSpeaker_Command = new shootSpeaker_Command();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 180);
    camera.setFPS(30);

    chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", null);
    chooser.addOption("Just Drive", auto_Drive_Command.withTimeout(RMap.autoDriveTime));
    chooser.addOption("Shoot & Drive", auto_Shoot_Command);
    chooser.addOption("Just Shoot", shootSpeaker_Command);
    SmartDashboard.putData(chooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    RMap.m_driveTrain_subsystem.setDefaultCommand(new teleDrive_Command());
    RMap.m_lift_subsystem.setDefaultCommand(new liftControl_Command());

    RMap.controller.a().onTrue(new shootSpeaker_Command());
    RMap.controller.x().onTrue(new pickupNote_Command());
    RMap.controller.b().onTrue(new shootAmp_Command());
    RMap.controller.y().onTrue(new bumpNote_Command());
  }

  public Command getAutoCommand() {
    return chooser.getSelected();
  }
}
