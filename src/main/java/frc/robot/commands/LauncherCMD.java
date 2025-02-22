package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class LauncherCMD extends Command {
    private final LauncherSubsystem m_Shooter;
    private final Timer m_timer;

    public LauncherCMD(LauncherSubsystem _Shooter) {
        this.m_Shooter = _Shooter;
        this.m_timer = new Timer();

        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_Shooter.runLauncher();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > 2;
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.stopLauncher();
    }
}
