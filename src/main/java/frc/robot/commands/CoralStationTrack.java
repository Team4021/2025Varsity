package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.VisionConstants;
import frc.robot.subsystems.vision.LimelightSubsystem;

/** Command that uses AprilTags to Align and Drive towards Coral Station */
public class CoralStationTrack extends Command {
  private LimelightSubsystem m_Limelight;

  // The subsystem the command runs on
  // private final LimelightSubsystem m_LimelightSubsystem;
  public CoralStationTrack(LimelightSubsystem m_Limelight) {
    // m_LimelightSubsystem = subsystem;
    this.m_Limelight = m_Limelight;
    addRequirements(m_Limelight);
  }

  @Override
  public void initialize() {
    VisionConstants.AlignmentConstants.driveTrackingAlignment = 0;
    VisionConstants.DistanceConstants.driveTrackingDistance = 0;
  }

  public void execute() {
    if (RobotContainer.m_LimelightSubsystem.coralStationIsTag()) {
      RobotContainer.m_LimelightSubsystem.calculateCoralStationX();
      RobotContainer.m_LimelightSubsystem.calculateCoralStationY();
    }
  }

  @Override
  public boolean isFinished() {
    VisionConstants.AlignmentConstants.driveTrackingAlignment = 0;
    VisionConstants.DistanceConstants.driveTrackingDistance = 0;
    return true;
  }
}
