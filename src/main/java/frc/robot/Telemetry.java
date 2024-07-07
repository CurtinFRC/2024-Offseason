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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Telemetry {
  /** Construct a telemetry object. */
  public Telemetry() {}

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");
  private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
  private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
  private final DoublePublisher omega = driveStats.getDoubleTopic("Omega").publish();
  private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
  private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();
  private final DoublePublisher rotation = driveStats.getDoubleTopic("Rotation").publish();
  StructArrayPublisher<SwerveModuleState> states =
      driveStats.getStructArrayTopic("SwerveModuleStates", SwerveModuleState.struct).publish();

  /* Keep a reference of the last pose to calculate the speeds */
  private Pose2d m_lastPose = new Pose2d();
  private double lastTime = Utils.getCurrentTimeSeconds();

  /* Accept the swerve drive state and telemeterize it to smartdashboard */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the pose */
    Pose2d pose = state.Pose;
    fieldTypePub.set("Field2d");
    fieldPub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});

    /* Telemeterize the robot's general speeds */
    double currentTime = Utils.getCurrentTimeSeconds();
    double diffTime = currentTime - lastTime;
    lastTime = currentTime;
    Transform2d poseDiff = pose.minus(m_lastPose);
    m_lastPose = pose;

    Translation2d velocities = poseDiff.getTranslation().div(diffTime);
    Rotation2d rotational_velocity = poseDiff.getRotation().div(diffTime);

    speed.set(velocities.getNorm());
    velocityX.set(velocities.getX());
    velocityY.set(velocities.getY());
    omega.set(rotational_velocity.getRadians());
    odomPeriod.set(state.OdometryPeriod);
    states.set(state.ModuleStates);
    rotation.set(state.Pose.getRotation().getRadians());
  }
}
