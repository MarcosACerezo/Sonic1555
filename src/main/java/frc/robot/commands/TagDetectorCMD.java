// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.LEDsSubsystem;
// import frc.robot.subsystems.LimelightSubsystem;

// public class TagDetectorCMD extends Command {
    
//     private final LimelightSubsystem m_Limelight;
//     private final LEDsSubsystem m_LEDs;

//     public TagDetectorCMD(LimelightSubsystem m_Limelight, LEDsSubsystem m_LEDs) {

//         this.m_LEDs = m_LEDs;
//         this.m_Limelight = m_Limelight;

//         addRequirements(m_LEDs);
//         addRequirements(m_Limelight);
        

//     }

//     @Override
//     public void initialize() {
        
//     }

//     @Override
//     public void execute() {
//         if (m_Limelight.AprilTagData() == 8 || m_Limelight.AprilTagData() == 7) {
//             m_LEDs.lightsNormal(Constants.LEDs.SpkrRead);
//         } else if (m_Limelight.AprilTagData() == 6) {
//             m_LEDs.lightsNormal(Constants.LEDs.AmpRead);
//         } else if (m_Limelight.AprilTagData() == 9) {
//             m_LEDs.lightsNormal(Constants.LEDs.SorcRead);
//         } else {
//             m_LEDs.lightsNormal(Constants.LEDs.NoTag);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }
//     // Empty blueprint 

// }
