package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.Vision;

/** Command that uses AprilTags to Align and Drive towards Coral Station */
public class leftCoralReefTrack extends Command {
  private Vision m_vision;
  public  double aprilTagAlignment;
  
    public  PIDController alignedPID =
             new PIDController(
                 VisionConstants.AlignmentConstants.kP,
                 VisionConstants.AlignmentConstants.kI,
                 VisionConstants.AlignmentConstants.kD);
       
         // The subsystem the command runs on
         // private final LimelightSubsystem m_LimelightSubsystem;
  public leftCoralReefTrack(Vision m_vision) {
           // m_LimelightSubsystem = subsystem;
           this.m_vision = m_vision;
           addRequirements(m_vision);
         }
       
         @Override
  public void initialize() {
           aprilTagAlignment = 0;  
           VisionConstants.AlignmentConstants.aprilTagAlignmentX = 0;
           VisionConstants.AlignmentConstants.aprilTagAlignmentY = 0;
         }
       
  public  void calculateAlignment() {
    aprilTagAlignment = alignedPID.calculate(m_vision.getTargetX(0).getRadians(), m_vision.calculateLeftCoralOffsetX());
    VisionConstants.AlignmentConstants.aprilTagAlignmentX = (-aprilTagAlignment * Constants.lastValidTargetAngle.getCos());
    VisionConstants.AlignmentConstants.aprilTagAlignmentY = (-aprilTagAlignment * Constants.lastValidTargetAngle.getSin());
  }

  @Override
  public boolean isFinished() {
    aprilTagAlignment = 0;
    VisionConstants.AlignmentConstants.aprilTagAlignmentX = 0;
    VisionConstants.AlignmentConstants.aprilTagAlignmentY = 0;
    return true;
  }
}
