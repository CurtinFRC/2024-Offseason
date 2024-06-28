// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ApplyChassisSpeeds;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private ChassisSpeeds m_lastSpeeds = new ChassisSpeeds();

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
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
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    /*
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     */
    /*
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
  }

  private ChassisSpeeds controlFunction(Pose2d pose, ChoreoTrajectoryState state) {
    var controller = new LTVUnicycleController(0.02, Constants.DrivebaseMaxSpeed);
    var speed = getState().speeds;
    var speedsDiff = speed.minus(m_lastSpeeds);
    var acceleration =
        Math.hypot(speedsDiff.vxMetersPerSecond, speedsDiff.vyMetersPerSecond) / 0.02;
    m_lastSpeeds = speed;
    return controller.calculate(pose, state.toTrajectoryState(acceleration));
  }

  /**
   * A command to follow the given Choreo trajectory.
   *
   * @param trajectoryName The name of the trajectory.
   * @param isRed Should the trajectory be from the red perspective.
   */
  public Command followTrajectory(String trajectoryName, boolean isRed) {
    return Choreo.choreoSwerveCommand(
        Choreo.getTrajectory(trajectoryName),
        () -> this.getState().Pose,
        this::controlFunction,
        (speeds) -> this.setControl(new ApplyChassisSpeeds().withSpeeds(speeds)),
        () -> isRed,
        this);
  }
}
