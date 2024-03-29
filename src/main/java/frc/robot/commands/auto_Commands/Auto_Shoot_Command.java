// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto_Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RMap;
import frc.robot.commands.shootSpeaker_Command;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shoot_Command extends SequentialCommandGroup {
  /** Creates a new Auto_Shoot. */
  public Auto_Shoot_Command() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootSpeaker_Command().withTimeout(3),
      new Auto_Drive_Command().withTimeout(RMap.autoShootAndDriveTime)
    );
  }
}
