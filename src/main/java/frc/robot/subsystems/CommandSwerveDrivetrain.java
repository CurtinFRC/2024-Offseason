// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.util.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
@SuppressWarnings("PMD.SingularField")
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  public enum RotationSetpoint {
    kAmp,
    kSpeaker,
  }

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Drive");
  private final StringPublisher m_activeCommand =
      driveStats.getStringTopic("Active Command").publish();
  private final BooleanPublisher m_activeCommandFinished =
      driveStats.getBooleanTopic("Active Command Finished").publish();
  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier;
  private double m_lastSimTime;
  private final PIDController m_xController = new PIDController(10, 0, 0.5);
  private final PIDController m_yController = new PIDController(10, 0, 0.5);
  private final PIDController m_rotationController = new PIDController(7, 0, 0.35);

  private final PIDController m_setpointRotationController = new PIDController(7, 0, 0.35);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    m_xController.setTolerance(0.005);
    m_yController.setTolerance(0.005);
    m_rotationController.setTolerance(0.005);
    m_setpointRotationController.setTolerance(0.005);

    configurePathPlanner();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    m_xController.setTolerance(0.05);
    m_yController.setTolerance(0.05);
    m_rotationController.setTolerance(0.05);

    configurePathPlanner();
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().orElse(Alliance.Blue)
                == Alliance.Red, // Assume the path needs to be flipped
        // for Red vs Blue, this is normally
        // the case
        this); // Subsystem for requirements
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective
     *
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     *
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     *
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     *
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }

    var currentcommand = getCurrentCommand();
    if (currentcommand != null) {
      m_activeCommand.set(currentcommand.toString());
      m_activeCommandFinished.set(currentcommand.isFinished());
    }
  }

  public Command getAutoPath(String pathName) {
    var initPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
    m_odometry.resetPosition(initPose.getRotation(), m_modulePositions, initPose);
    // return new PathPlannerAuto(pathName);
    return AutoBuilder.buildAuto(pathName);
  }

  public Command goToSetpoint(
      RotationSetpoint setpoint,
      SwerveRequest.FieldCentric drivetrain,
      CommandScheduler scheduler) {
    // Use AtomicReference to hold the target position, so it can be accessed in lambdas
    AtomicReference<Double> targetPosition = new AtomicReference<>(0.0);

    // Determine the target position based on the setpoint
    switch (setpoint) {
      case kAmp:
        targetPosition.set(-(Math.PI / 2)); // Example angle
        break;
      case kSpeaker:
        targetPosition.set(Math.PI); // Facing forward
        break;
      default:
        targetPosition.set(0.0);
        break;
    }

    // Define the command as a lambda that runs continuously until the robot reaches the target
    // position
    return Commands.run(
            () -> {
              // Continuously update the current position
              double currentPosition =
                  m_odometry.getEstimatedPosition().getRotation().getRadians() % (2 * Math.PI);
              double rotationOutput =
                  m_rotationController.calculate(currentPosition, targetPosition.get());

              // Apply the rotational output to the drivetrain
              drivetrain.withRotationalRate(rotationOutput);

              // Debug print to verify execution
              System.out.println("Moving towards setpoint...");

              // Specify when the command should stop (when the target position is reached)
            })
        .until(
            () ->
                Math.abs(
                        m_odometry.getEstimatedPosition().getRotation().getRadians()
                            - targetPosition.get())
                    < 0.01)
        .withTimeout(2)
        .andThen(
            () -> {
              // Stop the drivetrain when the target position is reached
              drivetrain.withRotationalRate(0);
              System.out.println("Target position reached.");
            });
  }
}
