package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RMap;
import frc.robot.subsystems.intake_subsystem;
import frc.robot.subsystems.shooter_subsystem;

public class shootAmp_Command extends Command {
    private intake_subsystem m_intake_subsystem;
    private shooter_subsystem m_shooter_subsystem;
    private CommandXboxController controller;
    private boolean cmd_finished;

    public shootAmp_Command() {
        this.m_intake_subsystem = RMap.m_intake_subsystem;
        this.m_shooter_subsystem = RMap.m_shooter_subsystem;
        this.controller = RMap.controller;

        addRequirements(m_intake_subsystem);
        addRequirements(m_shooter_subsystem);
    }

    @Override
    public void initialize() {
        cmd_finished = false;
    }

    @Override
    public void execute() {
        m_shooter_subsystem.setMotorsSpeed(RMap.ampSpeed);
        m_intake_subsystem.setMotor(RMap.intakeSpeed);

        
        if (!controller.b().getAsBoolean()) {
            m_shooter_subsystem.setMotorsSpeed(0);
            m_intake_subsystem.setMotor(0);
            cmd_finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter_subsystem.setMotorsSpeed(0);
        m_intake_subsystem.setMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return cmd_finished;
    }
}
