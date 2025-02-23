// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.OperatorInter;
import frc.robot.commands.ShootIntakeCMD;
import frc.robot.commands.LauncherCMD;
import frc.robot.commands.SensoredArmCMD;
import frc.robot.commands.SensoredIntakeCMD;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.utils.GamepadUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {

    private SwerveDriveSubsystem m_SwerveDrive = new SwerveDriveSubsystem();
    private final ArmSubsystem m_ShooterArm = new ArmSubsystem();
    private final IntakeSubsystem m_ShooterIntake = new IntakeSubsystem();
    private final LauncherSubsystem m_Shooter = new LauncherSubsystem();

    private final DigitalInput kTopSwitch = new DigitalInput(Constants.Sensors.kTopSwitchPort);
    private final DigitalInput kBottomSwitch = new DigitalInput(Constants.Sensors.kBottomSwitchPort);

    private XboxController m_DriveController = new XboxController(Constants.OperatorInter.DriverController);
    private XboxController m_ManipController = new XboxController(Constants.OperatorInter.ManipController);

    private final SendableChooser<Command> autoChooser;//TODO Find out if you need an autochooser object

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(new LauncherCMD(m_Shooter), new SequentialCommandGroup(new WaitCommand(1), new ShootIntakeCMD(m_ShooterIntake))));
        NamedCommands.registerCommand("Intake", new ParallelCommandGroup(new SensoredIntakeCMD(kTopSwitch, m_ShooterIntake), new SensoredArmCMD(kBottomSwitch, m_ShooterArm)));

        configureBindings();

        m_SwerveDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () ->
                    m_SwerveDrive.drive(
                        GamepadUtils.squareInput(
                            m_DriveController.getLeftY(), OperatorInter.kDriveDeadband),
                        GamepadUtils.squareInput(
                            m_DriveController.getLeftX(), OperatorInter.kDriveDeadband),
                        -GamepadUtils.squareInput(
                            m_DriveController.getRightX(), OperatorInter.kDriveDeadband),
                        true,
                        false,
                        Drivetrain.kMaxSpeedMPSRegular),
                m_SwerveDrive));
    
        m_ShooterArm.setDefaultCommand(new RunCommand(() -> m_ShooterArm.runAutomatic(), m_ShooterArm));
    }

    private void configureBindings() {
        // button to put swerve modules in an "x" configuration to hold position
        new JoystickButton(m_DriveController, XboxController.Button.kLeftStick.value)
            .whileTrue(new RunCommand(() -> m_SwerveDrive.setX(), m_SwerveDrive));

        new POVButton(m_ManipController, Constants.OperatorInter.kDPadUp)
            .whileTrue(new InstantCommand(() -> m_ShooterArm.setTargetPosition(Constants.Arm.kScoringPosition)));
        new POVButton(m_ManipController, Constants.OperatorInter.kDPadDown)
            .whileTrue(new InstantCommand(() -> m_ShooterArm.setTargetPosition(Constants.Arm.kIntakePosition)));
        new JoystickButton(m_ManipController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> m_ShooterArm.setTargetPosition(Constants.Arm.kClimbPosition)));

        new JoystickButton(m_ManipController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new RunCommand(() -> m_ShooterIntake.setPower(Constants.Intake.kIntakePower), m_ShooterIntake))
            .whileFalse(new RunCommand(() -> m_ShooterIntake.setPower(0), m_ShooterIntake));
        new JoystickButton(m_ManipController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand(() -> m_ShooterIntake.setPower(-Constants.Intake.kIntakePower), m_ShooterIntake))
            .whileFalse(new RunCommand(() -> m_ShooterIntake.setPower(0), m_ShooterIntake));
        
        new Trigger(() -> m_ManipController.getRightTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(() -> m_Shooter.runLauncher(), m_Shooter))
            .onFalse(new InstantCommand(() -> m_Shooter.stopLauncher(), m_Shooter));
        new Trigger(() -> m_ManipController.getLeftTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(() -> m_Shooter.runReverse(), m_Shooter))
            .onFalse(new InstantCommand(() -> m_Shooter.stopLauncher(), m_Shooter));

        new JoystickButton(m_ManipController, XboxController.Button.kX.value)
            .onTrue(new ParallelCommandGroup(new LauncherCMD(m_Shooter), new SequentialCommandGroup(new WaitCommand(1), new ShootIntakeCMD(m_ShooterIntake))));
        
        new JoystickButton(m_ManipController, XboxController.Button.kA.value)
            .onTrue(new ParallelCommandGroup(new SensoredIntakeCMD(kTopSwitch, m_ShooterIntake), new SensoredArmCMD(kBottomSwitch, m_ShooterArm)));
        // old driver button
        // new JoystickButton(m_DriveController, XboxController.Button.kX.value)
        //     .onTrue(new InstantCommand(() -> m_SwerveDrive.resetPose(Constants.Drivetrain.kDefaultPose)));
        // driver button test
        new JoystickButton(m_DriveController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> m_SwerveDrive.resetPose(Constants.Drivetrain.kDefaultPose)));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("BPose2 1Auto");
    }
}
