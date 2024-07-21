// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Telemetry {
  /** Construct a telemetry object. */
  public Telemetry() {}

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Drive");
  private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  private final DoublePublisher omega = driveStats.getDoubleTopic("Velocity Rotation").publish();
  private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
  private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();
  StructArrayPublisher<SwerveModuleState> states =
      driveStats.getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();
  private final StructPublisher<Pose2d> pose =
      driveStats.getStructTopic("Pose", Pose2d.struct).publish();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d current_pose = state.Pose;
    pose.set(current_pose);

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Transform2d poseDiff = current_pose.minus(m_lastPose);
    m_lastPose = current_pose;

    Translation2d velocities = poseDiff.getTranslation().div(diffTime);
    Rotation2d rotational_velocity = poseDiff.getRotation().div(diffTime);

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
    omega.set(rotational_velocity.getRadians());
    odomPeriod.set(state.OdometryPeriod);
    states.set(state.ModuleStates);
  }
}
