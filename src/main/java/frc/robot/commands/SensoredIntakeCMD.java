package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class SensoredIntakeCMD extends Command {
    private final IntakeSubsystem m_IntakeMotor;
    private final DigitalInput kTopSwitch;

    public SensoredIntakeCMD(DigitalInput _switch, IntakeSubsystem _IntakeMotor) {
        this.m_IntakeMotor = _IntakeMotor;
        this.kTopSwitch = _switch;

        addRequirements(m_IntakeMotor);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_IntakeMotor.setPower(-Constants.Intake.kIntakePower/2);
    }

    @Override
    public boolean isFinished() {
        return kTopSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeMotor.setPower(0.0);
    }
}