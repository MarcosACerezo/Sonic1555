package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSubsystem extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_ColorSensor;

    private int m_Distance;

    public ColorSubsystem() {
        m_ColorSensor = new ColorSensorV3(i2cPort);

        m_Distance = m_ColorSensor.getProximity();
    }

    public int getDistance() {
        m_Distance = m_ColorSensor.getProximity();
        return m_Distance;
    }

}
