package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark; 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDsSubsystem extends SubsystemBase {
    private Spark LEDs;

    public LEDsSubsystem() {
        LEDs = new Spark(0){};
    }

    public void lightsNormal(double tone) {
		LEDs.set(tone);
	}
  
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LEDs", LEDs.get());
    }
}