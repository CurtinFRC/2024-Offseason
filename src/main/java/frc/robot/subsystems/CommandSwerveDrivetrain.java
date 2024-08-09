// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
@SuppressWarnings("PMD.SingularField")
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private final NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private Pose3d m_botpose;
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier;
  private double m_lastSimTime;
  private final PIDController m_xController = new PIDController(10, 0, 0.5);
  private final PIDController m_yController = new PIDController(10, 0, 0.5);
  private final PIDController m_rotationController = new PIDController(7, 0, 0.35);
  private final ChoreoControlFunction m_swerveController;

  /* Orchestra classes */
  private final Orchestra m_orchestra = new Orchestra();
  private final ArrayList<String> m_songs = new ArrayList<String>();
  private static final AudioConfigs m_audioConfig =
      new AudioConfigs().withAllowMusicDurDisable(true);

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

  /**
   * Add the given songs from a chirp file.
   *
   * @param songs The name of the chirp files for the songs to add. Doesn't include file extension.
   */
  public void addMusic(String... songs) {
    m_songs.addAll(Arrays.asList(songs));
  }

  /**
   * Selects the track to play.
   *
   * @param track The selected track. Must be a loaded song.
   */
  public void selectTrack(String track) {
    if (!m_songs.contains(track)) {
      DataLogManager.log("Track " + track + " not found.");
    }

    frc.robot.Utils.logStatusCode(m_orchestra.loadMusic(track + ".chrp"));

    for (var i = 0; i < 4; i++) {
      var module = super.getModule(i);
      var driveMotor = module.getDriveMotor();
      var steerMotor = module.getSteerMotor();

      frc.robot.Utils.logStatusCode(driveMotor.getConfigurator().apply(m_audioConfig));
      frc.robot.Utils.logStatusCode(m_orchestra.addInstrument(driveMotor, 0));
      frc.robot.Utils.logStatusCode(steerMotor.getConfigurator().apply(m_audioConfig));
      frc.robot.Utils.logStatusCode(m_orchestra.addInstrument(steerMotor, 1));
    }
  }

  /**
   * A list with the loaded songs.
   *
   * @return The list of loaded songs.
   */
  public ArrayList<String> getSongs() {
    return m_songs;
  }

  public Orchestra getOrchestra() {
    return m_orchestra;
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

    if (DriverStation.getAlliance().isPresent()) {
      double[] data;
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        data = m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      } else {
        data = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
      }

      if ((int) data[7] < 2) {
        DataLogManager.log("Couldn't see enough tags, skipping vision measurement.");
        return;
      }

      m_botpose =
          new Pose3d(
              new Translation3d(data[0], data[1], data[2]),
              new Rotation3d(data[4], data[5], data[6]));

      if (reasonablePose(m_botpose)) {
        addVisionMeasurement(m_botpose.toPose2d(), frc.robot.Utils.now());
      }
    }
  }

  private boolean reasonablePose(Pose3d pose) {
    return !((Math.abs(pose.getZ()) > 0.1)
        && (pose.getX() > Constants.fieldX)
        && (pose.getX() < 0)
        && (pose.getY() > Constants.fieldY)
        && (pose.getY() < 0));
  }

  /**
   * Follow the given trajectory.
   *
   * @param name The name of the trajectory.
   * @param isRed The perspective of the trajectory.
   * @return A Command to follow the given trajectory.
   */
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
}
