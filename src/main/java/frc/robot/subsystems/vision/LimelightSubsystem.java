package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.VisionConstants;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable coralStationVision;
  NetworkTable leftReefVision;
  NetworkTable rightReefVision;
  public PIDController alignedPID =
      new PIDController(
          VisionConstants.AlignmentConstants.kP,
          VisionConstants.AlignmentConstants.kI,
          VisionConstants.AlignmentConstants.kD);
  public PIDController distancePID =
      new PIDController(
          VisionConstants.DistanceConstants.kP,
          VisionConstants.DistanceConstants.kI,
          VisionConstants.DistanceConstants.kD);

  public LimelightSubsystem() {
    coralStationVision = NetworkTableInstance.getDefault().getTable("Coral Station");
    leftReefVision = NetworkTableInstance.getDefault().getTable("leftReefVision");
    rightReefVision = NetworkTableInstance.getDefault().getTable("rightReefVision");
  }

  // Variables for coral station limelight
  public double coralStationX() {
    return coralStationVision.getEntry("tx").getDouble(0);
  }

  public double coralStationY() {
    return coralStationVision.getEntry("ty").getDouble(0);
  }

  public double coralStationA() {
    return coralStationVision.getEntry("ta").getDouble(0);
  }

  public boolean coralStationIsTag() {
    return (coralStationVision.getEntry("tv").getDouble(0) == 1);
  }

  public void calculateCoralStationX() {
    VisionConstants.AlignmentConstants.driveTrackingAlignment =
        alignedPID.calculate(
            RobotContainer.m_LimelightSubsystem.coralStationX(),
            VisionConstants.CoralStationConstants.kAlignedSetpoint);
  }

  public void calculateCoralStationY() {
    VisionConstants.CoralStationConstants.coralStationActualDistance =
        ((VisionConstants.CoralStationConstants.kTargetHeight
                - VisionConstants.LimelightMount.kCameraHeight)
            / Math.tan(
                VisionConstants.LimelightMount.kCameraAngleRad
                    + RobotContainer.m_LimelightSubsystem.coralStationY()));
    VisionConstants.DistanceConstants.driveTrackingDistance =
        distancePID.calculate(
            VisionConstants.CoralStationConstants.coralStationActualDistance,
            VisionConstants.CoralStationConstants.kDistanceSetpoint);
  }

  // Variables for left reef limelight
  public double leftReefX() {
    return leftReefVision.getEntry("tx").getDouble(0);
  }

  public double leftReefY() {
    return leftReefVision.getEntry("ty").getDouble(0);
  }

  public double leftReefA() {
    return leftReefVision.getEntry("ta").getDouble(0);
  }

  public boolean leftReefIsTag() {
    return (leftReefVision.getEntry("tv").getDouble(0) == 1);
  }

  // Variables for right reef limelight
  public double rightReefX() {
    return rightReefVision.getEntry("tx").getDouble(0);
  }

  public double rightReefY() {
    return rightReefVision.getEntry("ty").getDouble(0);
  }

  public double rightReefA() {
    return rightReefVision.getEntry("ta").getDouble(0);
  }

  public boolean rightReefIsTag() {
    return (rightReefVision.getEntry("tv").getDouble(0) == 1);
  }
}
