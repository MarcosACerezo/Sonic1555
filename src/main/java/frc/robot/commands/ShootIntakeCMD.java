package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootIntakeCMD extends Command {
    private final IntakeSubsystem m_Intake;
    private final Timer m_Timer;

    public ShootIntakeCMD(IntakeSubsystem _Intake) {
        this.m_Intake = _Intake;
        this.m_Timer = new Timer();

        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        m_Timer.start();
    }

    @Override
    public void execute() {
        m_Intake.setPower(-Constants.Intake.kIntakePower);
    }

    @Override
    public boolean isFinished() {
        return m_Timer.get() > 2;
    }

    @Override
    public void end(boolean interrupted) {
        m_Intake.setPower(0);
    }
}
