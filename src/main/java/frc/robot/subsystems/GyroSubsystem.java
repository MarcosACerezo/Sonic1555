// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class GyroSubsystem extends SubsystemBase {
//   /** Creates a new Gyro. */
//   private AnalogGyro m_gyro;
//   public GyroSubsystem() {
//     m_gyro = new AnalogGyro(0);
//     m_gyro.calibrate();

//   }



//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.getNumber("Gyro angle", m_gyro.getAngle());
//   }

// // Use gyro declaration from above here


// }