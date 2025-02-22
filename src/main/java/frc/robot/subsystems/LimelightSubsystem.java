// package frc.robot.subsystems;

// import org.opencv.core.Mat;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class LimelightSubsystem extends SubsystemBase {
//     Thread m_visionThread;
//     double m_AprilTagValue;

//     public LimelightSubsystem() {
//         m_visionThread = new Thread(() -> {
//             UsbCamera m_camera = CameraServer.startAutomaticCapture();//turns on camera
//             m_camera.setFPS(30);
//             m_camera.setResolution(10, 10);

//             CvSink cvsink = CameraServer.getVideo();
//             CvSource cvsource = CameraServer.putVideo("Rectangle", 160, 120);
//             Mat mat = new Mat(); //From Wii Sports?!
//         });
//         m_visionThread.setDaemon(true);
//         m_visionThread.start();
//     }

//     public double AprilTagData() {
//         m_AprilTagValue = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
//         return m_AprilTagValue;
//     }

//     @Override
//     public void periodic() { // prints information for what AprilTags are being read
//         SmartDashboard.putNumber("AprilTags", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0));
        
//     }
// }
