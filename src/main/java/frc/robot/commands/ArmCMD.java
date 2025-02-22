package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCMD extends Command {
    private final ArmSubsystem m_Arm;
    private final double kArmPos;

    public ArmCMD(ArmSubsystem _Arm, double _ArmPos) {
        this.m_Arm = _Arm;
        this.kArmPos = _ArmPos;

        addRequirements(m_Arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_Arm.setTargetPosition(kArmPos);
        m_Arm.runAutomatic();
    }

    @Override
    public boolean isFinished() {
        return m_Arm.isDoneMoving();
    }

    @Override
    public void end(boolean interrupted) {}
}
