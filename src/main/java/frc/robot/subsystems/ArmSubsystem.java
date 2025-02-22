// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//changed imports  reasoning
//motor controller class
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;//Controller Type
//used for configurations of the spark objects
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//used for configurations of the spark objects
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkLowLevel;

public class ArmSubsystem extends SubsystemBase {
    private SparkMaxConfig config;
    private SparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_controller;
    private double m_setpoint;

    private TrapezoidProfile m_profile;
    private Timer m_timer;
    private TrapezoidProfile.State m_startState;
    private TrapezoidProfile.State m_endState;

    private TrapezoidProfile.State m_targetState;
    private double m_feedforward;
    private double m_manualValue;

    /** Creates a new ArmSubsystem. */
    public ArmSubsystem() {
        //creates a new configuration for the arm SparkMax object
        /**The new config class handles the configurations for all 
         * encoder, motor controller and controller type configurations*/
        config = new SparkMaxConfig();
        //sparkmax configurations
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(Constants.Arm.kCurrentLimit);
        //closed loop configurations sets PID Values each value is a double
        config.closedLoop
            .pid(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD);
        // set up the motor encoder conversion factors to convert to radians and radians per
        // second for position and velocity
        config.encoder  
            .positionConversionFactor(Constants.Arm.kPositionFactor)
            .velocityConversionFactor(Constants.Arm.kVelocityFactor);
        //softLimitConfigurations
        config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit((float) Constants.Arm.kSoftLimitForward)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit((float) Constants.Arm.kSoftLimitReverse);

        //Creating Sparkmax, encoder, and Closed loop objects
        m_motor = new SparkMax(Constants.Arm.kArmCanId, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_controller = m_motor.getClosedLoopController();

        /**VERY IMPORTANT 
         * ResetSafe - this parameter resets all values to default values for the spark max, 
         * and then writes in the new values, if resetSafe is false, then the new values will
         * be added, but any value you aren't writing over will remain the same and could not 
         * be the default value
         * PersistMode - needs to be true to save the configurations you added after you use the robot */
        m_encoder.setPosition(0.0);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_setpoint = Constants.Arm.kHomePosition;
        m_timer = new Timer();
        m_timer.start();
        updateMotionProfile();
    }

    /**
     * Sets the target position and updates the motion profile if the target position changed.
     *
     * @param _setpoint The new target position in radians.
     */
    public void setTargetPosition(double _setpoint) {
        if (_setpoint != m_setpoint) {
            m_setpoint = _setpoint;
            System.out.println(m_setpoint);
            updateMotionProfile();
        }
    }

    /**
     * Update the motion profile variables based on the current setpoint and the pre-configured motion
     * constraints.
     */
    private void updateMotionProfile() {
        m_startState = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
        m_endState = new TrapezoidProfile.State(m_setpoint, 0.0);
        m_profile = new TrapezoidProfile(Constants.Arm.kArmMotionConstraint);
        m_timer.reset();
    }

    /**
     * Drives the arm to a position using a trapezoidal motion profile. This function is usually
     * wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
     *
     * <p>This function updates the motor position control loop using a setpoint from the trapezoidal
     * motion profile. The target position is the last set position with {@code setTargetPosition}.
     */
    public void runAutomatic() {
        double elapsedTime = m_timer.get();
        if (m_profile.isFinished(elapsedTime)) {
            m_targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
        } else {
            m_targetState = m_profile.calculate(elapsedTime, m_startState, m_endState);
        }

        m_feedforward =
            Constants.Arm.kArmFeedforward.calculate(
                m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
        m_controller.setReference(
            m_targetState.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, m_feedforward);
    }

    /**
     * Drives the arm using the provided power value (usually from a joystick). This also adds in the
     * feedforward value which can help counteract gravity.
     *
     * @param _power The motor power to apply.
     */
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        m_setpoint = m_encoder.getPosition();
        updateMotionProfile();
        // update the feedforward variable with the newly zero target velocity
        m_feedforward =
            Constants.Arm.kArmFeedforward.calculate(
                m_encoder.getPosition() + Constants.Arm.kArmZeroCosineOffset, m_targetState.velocity);
        // set the power of the motor
        m_motor.set(_power + (m_feedforward / 12.0));
        m_manualValue = _power; // this variable is only used for logging or debugging if needed
    }

    public double armPosition() {
        return m_encoder.getPosition();
    }

    public boolean isDoneMoving() {
        return m_profile.isFinished(m_timer.get()); 
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
        SmartDashboard.putNumber("ARM PLACE", m_feedforward);
        SmartDashboard.putNumber("Arm Power", m_motor.get());
    }
}