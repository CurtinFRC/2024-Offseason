// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
@SuppressWarnings("PMD.SingularField")
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier;
  private double m_lastSimTime;

  private final PIDController m_xController = new PIDController(10, 0, 0.5);
  private final PIDController m_yController = new PIDController(10, 0, 0.5);
  private final PIDController m_rotationController = new PIDController(7, 0, 0.35);
  private final ChoreoControlFunction m_swerveController;

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

    m_swerveController =
        Choreo.choreoSwerveController(m_xController, m_yController, m_rotationController);
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

    m_swerveController =
        Choreo.choreoSwerveController(m_xController, m_yController, m_rotationController);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
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

  public Command followTrajectory(String name, boolean isRed) {
    var traj = Choreo.getTrajectory(name);
    var initPose = traj.getInitialPose();
    m_odometry.resetPosition(initPose.getRotation(), m_modulePositions, initPose);

    return Choreo.choreoSwerveCommand(
        traj,
        () -> getState().Pose,
        m_swerveController,
        (speeds) -> setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
        () -> isRed,
        this);
  }

  @Override
  public void simulationPeriodic() {
    updateSimState(0.020, RobotController.getBatteryVoltage());
  }
}
