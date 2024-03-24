package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RMap;

public class shootAmp_Command extends Command {
    private boolean cmd_finished;

    public shootAmp_Command() {
        addRequirements(RMap.m_intake_subsystem);
        addRequirements(RMap.m_shooter_subsystem);
    }

    @Override
    public void initialize() {
        cmd_finished = false;
    }

    @Override
    public void execute() {
        RMap.m_shooter_subsystem.setMotorsSpeed(RMap.ampSpeed);
        RMap.m_intake_subsystem.setMotor(RMap.intakeSpeed);

        RMap.controller.b().onFalse(new InstantCommand(() -> cmd_finished = true));
    }

    @Override
    public void end(boolean interrupted) {
        RMap.m_shooter_subsystem.setMotorsSpeed(0);
        RMap.m_intake_subsystem.setMotor(0);
    }
    
    @Override
    public boolean isFinished() {
        return cmd_finished;
    }
}
