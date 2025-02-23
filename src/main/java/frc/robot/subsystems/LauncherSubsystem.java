package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    private SparkMax m_topMotor;
    private SparkMax m_bottomMotor;
    private SparkMaxConfig config;
    private double m_launcherRunning;

    /** Creates a new LauncherSubsystem. */
    public LauncherSubsystem() {
        // create two new SPARK MAXs and configure them
        config = new SparkMaxConfig();
        //both motors use the same intake parameters
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.Launcher.kCurrentLimit);

        m_topMotor =
            new SparkMax(Constants.Launcher.kTopCanId, MotorType.kBrushless);
        m_bottomMotor =
            new SparkMax(Constants.Launcher.kBottomCanId, MotorType.kBrushless);

        m_topMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_launcherRunning = 0;
    }

    /**
     * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
     * in a {@code RunCommand}.
     */
    public void runLauncher() {
        m_launcherRunning = 1;
    }

    public void runReverse() {
        m_launcherRunning = -1;
    }

    /**
     * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
     * in a {@code RunCommand}.
     */
    public void stopLauncher() {
        m_launcherRunning = 0;
    }

    @Override
    public void periodic() { // this method will be called once per scheduler run
        // set the launcher motor powers based on whether the launcher is on or not
        if (m_launcherRunning == 1) {
            m_topMotor.set(Constants.Launcher.kTopPower);
            m_bottomMotor.set(Constants.Launcher.kBottomPower);
        } else if (m_launcherRunning == -1) {
            m_topMotor.set(-Constants.Launcher.kTopPower);
            m_bottomMotor.set(-Constants.Launcher.kBottomPower);
        } else {
            m_topMotor.set(0.0);
            m_bottomMotor.set(0.0);
        }
    }
}