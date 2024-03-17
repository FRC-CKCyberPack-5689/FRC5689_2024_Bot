// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap;
import frc.robot.subsystems.driveTrain_subsystem;
import frc.robot.subsystems.intake_subsystem;
import frc.robot.subsystems.shooter_subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shoot_Command extends SequentialCommandGroup {
  /** Creates a new Auto_Shoot. */
  public Auto_Shoot_Command(shooter_subsystem m_shooter_subsystem, driveTrain_subsystem mDriveTrain_subsystem, intake_subsystem mIntake_subsystem, CommandXboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootSpeaker_Command(m_shooter_subsystem, mIntake_subsystem).withTimeout(6),
      new Auto_Drive_Command(mDriveTrain_subsystem).withTimeout(RMap.autoDriveTime*2)
    );
  }
}
