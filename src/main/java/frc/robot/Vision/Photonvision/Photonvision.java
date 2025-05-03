// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision.Photonvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drivetrain;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Photonvision extends SubsystemBase {

  private String photonCameraName;
  private PhotonCamera camera;

  private PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded),
          PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS,
          new Transform3d());

  private EstimatedRobotPose estimatedRobotPose;
  private StructPublisher<Pose2d> photonvisionRobotPose;

  private Optional<PhotonTrackedTarget> currentTrackedTarget;
  private Optional<PhotonPipelineResult> currentPipelineResult;
  private boolean hasTarget;

  private Drivetrain drivetrain = Drivetrain.getInstance();

  public Photonvision(String photonCameraName) {
    this.photonCameraName = photonCameraName;
    camera = new PhotonCamera(photonCameraName);

    this.photonvisionRobotPose =
        NetworkTableInstance.getDefault()
            .getTable("Goldfish")
            .getStructTopic("Photonvision Vision Pose", Pose2d.struct)
            .publish();
  }

  public Photonvision(String photonCameraName, Transform3d robotRelativeToCamera) {
    this(photonCameraName);
    photonPoseEstimator.setRobotToCameraTransform(robotRelativeToCamera);
  }

  public Optional<PhotonPipelineResult> getNewestResult() {
    var photonResults = camera.getAllUnreadResults();
    if (!photonResults.isEmpty()) {
      return Optional.of(photonResults.get(photonResults.size() - 1));
    }
    return Optional.empty();
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return currentPipelineResult.isPresent()
        ? Optional.of(currentPipelineResult.get().getBestTarget())
        : Optional.empty();
  }

  public Optional<Transform3d> getObjectRelativeTransform() {
    return currentTrackedTarget.isPresent()
        ? Optional.of(currentTrackedTarget.get().getBestCameraToTarget())
        : Optional.empty();
  }

  public boolean hasTarget() {
    return currentTrackedTarget.isPresent();
  }

  public double getDistanceToObjectPose() {
    return PhotonUtils.getDistanceToPose(drivetrain.getPose2d(), getObjectFieldRelativePose());
  }

  public double getPoseAmbiguity() {
    return currentTrackedTarget.isPresent() ? currentTrackedTarget.get().getPoseAmbiguity() : 1;
  }

  public double getSkew() {
    return currentTrackedTarget.isPresent() ? currentTrackedTarget.get().getSkew() : 0;
  }

  public double getYaw() {
    return currentTrackedTarget.isPresent() ? currentTrackedTarget.get().getYaw() : 0;
  }

  public double getPitch() {
    return currentTrackedTarget.isPresent() ? currentTrackedTarget.get().getPitch() : 0;
  }

  public double getFiducialId() {
    return currentTrackedTarget.isPresent() ? currentTrackedTarget.get().getFiducialId() : -1;
  }

  public void addVisionMeasurement() {
    drivetrain.addVisionMeasurement(estimatedRobotPose);
  }

  public void addEstimatorHeadingData() {
    photonPoseEstimator.addHeadingData(Timer.getTimestamp(), drivetrain.getHeadingRotation2d());
  }

  public void updatePoseEstimator() {
    if (currentPipelineResult.isPresent()) {
      Optional<EstimatedRobotPose> estimatedRobotPose =
          photonPoseEstimator.update(currentPipelineResult.get());
      if (estimatedRobotPose.isPresent()) {
        this.estimatedRobotPose = estimatedRobotPose.get();
      }
    }
  }

  public Pose3d getPoseEstimate3d() {
    return this.estimatedRobotPose.estimatedPose;
  }

  public Pose2d getPoseEstimate2d() {
    return getPoseEstimate3d().toPose2d();
  }

  /**
   * Only use if drivetrain odometry is already sufficently accurate
   *
   * @return Pose2d
   */
  public Pose2d getObjectFieldRelativePose() {
    var relativeTransform = getObjectRelativeTransform();
    return relativeTransform.isPresent()
        ? new Pose3d(drivetrain.getPose2d()).plus(relativeTransform.get()).toPose2d()
        : drivetrain.getPose2d();
  }

  @Override
  public void periodic() {

    currentPipelineResult = getNewestResult();
    currentTrackedTarget = getBestTarget();
    hasTarget = hasTarget();

    updatePoseEstimator();
    addEstimatorHeadingData();
    photonvisionRobotPose.set(getPoseEstimate2d());

    SmartDashboard.putBoolean(photonCameraName.concat("/Has Target"), hasTarget);
    SmartDashboard.putNumber(
        photonCameraName.concat("/Distance to Target"), getDistanceToObjectPose());
    SmartDashboard.putNumber(photonCameraName.concat("/Target Skew"), getSkew());
    SmartDashboard.putNumber(photonCameraName.concat("/Target Pitch"), getPitch());
    SmartDashboard.putNumber(photonCameraName.concat("/Target Yaw"), getYaw());
  }
}
