package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class LEDs {
        public static final double NoTag = 0.61; // Limelight not seeing any tags
        public static final double RunAuto = 0.77; // LimeLight being a good little boy.
        public static final double SpkrRead = 0.87; // LimeLight sees the Speaker AprilTags
        public static final double AmpRead = 0.93; // Limelight sees the Amp AprilTag
        public static final double SorcRead = 0.57; // Limelight sees the Source AprilTag
    }

    public static final class OperatorInter {
        // // original controller assignment
        // public static final int DriverController = 1;
        // public static final int ManipController = 0;
        // controller assignment test
        public static final int DriverController = 1;
        public static final int ManipController = 1;
        
        public static final double kDriveDeadband = 0.05;
        public static final double kArmManualDeadband = 0.05;
        public static final double kArmManualScale = 0.5;

        public static final int kDPadUp = 0;
        public static final int kDPadDown = 180;
        public static final int kDPadLeft = 270;
        public static final int kDPadRight = 90;
    }

    public static final class Drivetrain {
        // speed limiters to make stuff move at certain speeds.
        public static final double kMaxSpeedMPSRegular = 4.8; // regular speed mode; return to 4.8 eventually
        public static final double kMaxSpeedMPSSlow = 1.0; // used in super slow mode, for finer control
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(21.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(21.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightDrivingCanId = 17;

        public static final int kFrontLeftTurningCanId = 10;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kRearRightTurningCanId = 16;

        public static final boolean kGyroReversed = false;
        
        public static final Pose2d kDefaultPose = new Pose2d(0, 0, new Rotation2d(0.0));
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class Intake {
        public static final int kCanId = 21;
        public static final boolean kMotorInverted = true;
        public static final int kCurrentLimit = 80;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
        public static final double kPositionTolerance = 0.5;
        public static final double kIntakePower = 0.7;
        public static final double kRetractDistance = -3.5;
        public static final double kShotFeedTime = 1.0;
    }
    
    public static final class Launcher {
        public static final int kTopCanId = 18;
        public static final int kBottomCanId = 19;

        public static final int kCurrentLimit = 80;

        public static final double kTopPower = -0.8; // apparently, top and bottom are switched for the launch motors
        public static final double kBottomPower = -0.9;
    }

    public static final class Arm {
        public static final int kArmCanId = 20;
        public static final boolean kArmInverted = true;
        public static final int kCurrentLimit = 40;

        public static final double kSoftLimitReverse = -1.19;
        public static final double kSoftLimitForward = 0.1;

        public static final double kArmGearRatio = (1.0 / 75.0) * (28.0 / 50.0) * (16.0 / 64.0);
        public static final double kPositionFactor =
            kArmGearRatio
                * 2.0
                * Math.PI; // multiply SM value by this number and get arm position in radians
        public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
        public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
        public static final double kArmZeroCosineOffset =
            1.342; // radians to add to converted arm position to get real-world arm position (starts at
        // ~76.9deg angle)
        public static final ArmFeedforward kArmFeedforward =
            new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
        public static final double kP = 1.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final TrapezoidProfile.Constraints kArmMotionConstraint =
            new TrapezoidProfile.Constraints(3.5, 1.5);

        public static final double kHomePosition = 0.0;
        public static final double kScoringPosition = 0.0;
        // public static final double kIntakePosition = -1.30;
        public static final double kIntakePosition = -1.08;
        public static final double kClimbPosition = 0.05;
    }

    public static final class Auton {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Sensors {
        public static final int kTopSwitchPort = 0;
        public static final int kBottomSwitchPort = 1;
    }
}
