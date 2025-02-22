package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class SensoredArmCMD extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final DigitalInput kBottomSwitch;

    public SensoredArmCMD(DigitalInput _switch, ArmSubsystem _ArmSubsystem) {
        this.m_ArmSubsystem = _ArmSubsystem;
        this.kBottomSwitch = _switch;

        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_ArmSubsystem.setTargetPosition(Constants.Arm.kIntakePosition);
        m_ArmSubsystem.runAutomatic();
    }

    @Override
    public boolean isFinished() {
        return kBottomSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setTargetPosition(Constants.Arm.kScoringPosition);
        m_ArmSubsystem.runAutomatic();
    }
}
