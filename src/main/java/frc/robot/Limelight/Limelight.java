package frc.robot.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drivetrain;

public class Limelight extends SubsystemBase {
    
    private String networkTableName;
    private boolean hasTag;
    private double tagDistance;
    private Pose2d robotPose;
    private StructPublisher<Pose2d> limelightRobotPose;

    public Limelight(String networkTableName) {

        this.networkTableName = networkTableName;
        this.hasTag = false;
        this.tagDistance = 0.0;
        this.robotPose = new Pose2d();

        this.limelightRobotPose = 
            NetworkTableInstance.getDefault()
            .getTable("Goldfish")
            .getStructTopic("Vision Pose", Pose2d.struct)
            .publish();

    }

    public Limelight(String networkTableName, Pose3d limelightPose) {
        this(networkTableName);
        this.setLimelightPose(limelightPose);
    }

    public void setLimelightPose(Pose3d limelightPose) {

        LimelightHelpers.setCameraPose_RobotSpace(
            networkTableName,
            limelightPose.getX(), 
            limelightPose.getY(), 
            limelightPose.getZ(), 
            Units.radiansToDegrees(limelightPose.getRotation().getX()), 
            Units.radiansToDegrees(limelightPose.getRotation().getY()), 
            Units.radiansToDegrees(limelightPose.getRotation().getZ()));
    }

    public Pose2d getRobotPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(networkTableName);
    }

    public LimelightHelpers.PoseEstimate getRobotPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTableName);
    }

    public boolean getTagVisible() {
        return LimelightHelpers.getTV(networkTableName);
    }

    public int getID() {
        return (int) LimelightHelpers.getFiducialID(networkTableName);
    }

    public double getTagDistance() {
        return tagDistance;
    }

    //Horizontal Angle to Tag
    public double getTx() {
        return LimelightHelpers.getTX(networkTableName);
    }

    //Vertical Angle to Tag
    public double getTy() {
        return LimelightHelpers.getTY(networkTableName);
    }

    public double getArea() {
        return LimelightHelpers.getTA(networkTableName);
    }

    public String getNetworkTableName() {
        return networkTableName;
    }

    public boolean isConnected() {
        return NetworkTableInstance.getDefault().getTable(networkTableName) == null;
    }

    @Override
    public void periodic() {
        LimelightHelpers.PoseEstimate poseEstimate = this.getRobotPoseEstimate();

        if (poseEstimate == null) {
            this.robotPose = new Pose2d(0, 0, new Rotation2d(0, 0));
            this.tagDistance = 0.0;
        } else {
            this.robotPose = poseEstimate.pose;
            this.tagDistance = poseEstimate.avgTagDist;
            Drivetrain.getInstance().addVisionMeasurement(poseEstimate);
            limelightRobotPose.set(robotPose);
        }

        this.hasTag = this.getTagVisible();
        SmartDashboard.putBoolean("Limelight " + networkTableName + "/Tag In View", hasTag);
        SmartDashboard.putNumber("Limelight " + networkTableName + "/Tag Distance", tagDistance);

    }

}
