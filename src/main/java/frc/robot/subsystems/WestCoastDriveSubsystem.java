// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.MathUtil;
// //import edu.wpi.first.math.controller.PIDController;
// import frc.robot.Constants;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkRelativeEncoder;

// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class WestCoastDriveSubsystem extends SubsystemBase {
//   private CANSparkMax m_frontLeftMotor;
//   private CANSparkMax m_frontRightMotor;
//   private CANSparkMax m_rearLeftMotor;
//   private CANSparkMax m_rearRightMotor;

//   private double outputSpeedL;
//   private double errorL;
//   private double sensorPositionL;
//   private double Position;
//   private double outputSpeed;
//   private double error;

//   private RelativeEncoder m_frontLeftEncoder;
//   private RelativeEncoder m_frontRightEncoder;

//   private ADIS16470_IMU m_gyro;

//   /** Creates a new DrivetrainSubsystem. */
//   public WestCoastDriveSubsystem() {
//     m_frontLeftMotor  = new CANSparkMax(Constants.Drivetrain.kFrontLeftCanId, CANSparkLowLevel.MotorType.kBrushless);
//     m_frontLeftMotor.setInverted(Constants.Drivetrain.kFrontLeftInverted);
//     m_frontLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
//     //Hey, encoder and stuff
//     m_frontLeftEncoder = m_frontLeftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

//     m_frontLeftMotor.burnFlash();

//     m_frontRightMotor = new CANSparkMax(Constants.Drivetrain.kFrontRightCanId, CANSparkLowLevel.MotorType.kBrushless);
//     m_frontRightMotor.setInverted(Constants.Drivetrain.kFrontRightInverted);
//     m_frontRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
    
//     m_frontRightEncoder = m_frontLeftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
//     //Hey, encoder and stuff
//     m_frontRightEncoder = m_frontRightMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
//     m_frontRightMotor.burnFlash();

//     m_rearLeftMotor   = new CANSparkMax(Constants.Drivetrain.kRearLeftCanId, CANSparkLowLevel.MotorType.kBrushless);
//     m_rearLeftMotor.setInverted(Constants.Drivetrain.kRearLeftInverted);
//     m_rearLeftMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
//     m_rearLeftMotor.burnFlash();

//     m_rearRightMotor  = new CANSparkMax(Constants.Drivetrain.kRearRightCanId, CANSparkLowLevel.MotorType.kBrushless);
//     m_rearRightMotor.setInverted(Constants.Drivetrain.kRearRightInverted);
//     m_rearRightMotor.setSmartCurrentLimit(Constants.Drivetrain.kCurrentLimit);
//     m_rearRightMotor.burnFlash();
//   }

//   public void driveArcade(double _straight, double _turn) {
//     double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
//     double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);
    
//     m_frontLeftMotor.set(left);
//     m_frontRightMotor.set(right);
//     m_rearLeftMotor.set(left);
//     m_rearRightMotor.set(right);
//   }

//   public void tankDrive(double left, double right) {
//       m_frontLeftMotor.set(left * Constants.Drivetrain.speedLimit) ;
//       m_frontRightMotor.set(right * Constants.Drivetrain.speedLimit) ;
//       m_rearLeftMotor.set(left * Constants.Drivetrain.speedLimit) ;
//       m_rearRightMotor.set(right * Constants.Drivetrain.speedLimit) ;
//   }

//   public void resetEncoders(){
//     m_frontLeftEncoder.setPosition(0);
//     m_frontRightEncoder.setPosition(0);
//   }

//   public double getEncoder(){
//     return m_frontLeftEncoder.getPosition();
//   }

//   public void DistancePID(double setpoint){
//     //Get encoder position Left
//     sensorPositionL = m_frontLeftEncoder.getPosition() * Constants.Drivetrain.kDistanceConversion;

//     //Calculations Left
//     double currentL = (setpoint / 4) - m_frontLeftEncoder.getPosition();
//     errorL = currentL - sensorPositionL;
//     double dtL = Timer.getFPGATimestamp() - Constants.Drivetrain.lastTimeStampL;

//     if(Math.abs(errorL) < Constants.Drivetrain.ilimit){
//       Constants.Drivetrain.errorSumL += errorL*dtL;
//     }

//     double errorRateL = (errorL - Constants.Drivetrain.lastErrorL) / dtL;
//     outputSpeedL = Constants.Drivetrain.kP * errorL + Constants.Drivetrain.kI * Constants.Drivetrain.errorSumL + Constants.Drivetrain.kD * errorRateL;

//     //Output to motors
//     m_frontLeftMotor.set(outputSpeedL / 1.75);
//     m_frontRightMotor.set(outputSpeedL / 2);
//     m_rearLeftMotor.set(outputSpeedL / 1.75);
//     m_rearRightMotor.set(outputSpeedL / 2);

//     //Update variables
//     Constants.Drivetrain.lastTimeStampL = Timer.getFPGATimestamp();
//     Constants.Drivetrain.lastErrorL = errorL;
//   }


// //   public void TurnPID(double angle){
// //     //Get turn position
// //     Position = m_gyro.getAngle();
    
// //     double current = (angle) - m_gyro.getAngle();
// //     error = current - Position;
// //     double dt = Timer.getFPGATimestamp() - Constants.Drivetrain.lastTimeStamp;

// //     if(Math.abs(error) < Constants.Drivetrain.ilimit){
// //       Constants.Drivetrain.errorSum += error*dt;
// //     }

// //     double errorRate = (error - Constants.Drivetrain.lastError) / dt;
// //     outputSpeed = Constants.Drivetrain.turnP * error + Constants.Drivetrain.turnI * Constants.Drivetrain.errorSum + Constants.Drivetrain.turnD * errorRate;

// //     //Output to motors
// //     m_frontLeftMotor.set(outputSpeed / -1.75);
// //     m_frontRightMotor.set(outputSpeed / 2);
// //     m_rearLeftMotor.set(outputSpeed / -1.75);
// //     m_rearRightMotor.set(outputSpeed / 2);

// //     //Update variables
// //     Constants.Drivetrain.lastTimeStamp = Timer.getFPGATimestamp();
// //     Constants.Drivetrain.lastError = error;

// //   }

//   public void resetYAW(){
//     m_gyro.reset();
//   }

//   public boolean madeIt(double setpoint){
//     return Math.abs(m_frontLeftEncoder.getPosition() - setpoint / 4) < 0.1;

//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }

//   @Override
//   public void initSendable(SendableBuilder builder) {
//     super.initSendable(builder);
//   }

//   public boolean angleMade(Double angle) {
//     return Math.abs(m_gyro.getYComplementaryAngle()) < 0.1;
//   }

// }