// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.util.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
@SuppressWarnings("PMD.SingularField")
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Drive");
  private final DoublePublisher m_target = driveStats.getDoubleTopic("Module/Target").publish();
  private final DoublePublisher m_position = driveStats.getDoubleTopic("Module/Position").publish();
  private final DoublePublisher m_error = driveStats.getDoubleTopic("Module/Error").publish();
  private final DoublePublisher m_output = driveStats.getDoubleTopic("Module/Output").publish();
  private final StringPublisher m_activeCommand =
      driveStats.getStringTopic("Active Command").publish();
  private final StructPublisher m_limelightPose =
      driveStats.getStructTopic("Limelight Pose", Pose2d.struct).publish();
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

  private final SysIdRoutine[] m_driveSysIdRoutines = new SysIdRoutine[4];
  private final SysIdRoutine[] m_steerSysIdRoutines = new SysIdRoutine[4];

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    int[] validIDs = {3, 4};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

    m_xController.setTolerance(0.005);
    m_yController.setTolerance(0.005);
    m_rotationController.setTolerance(0.005);

    m_swerveController =
        Choreo.choreoSwerveController(m_xController, m_yController, m_rotationController);

    configurePathPlanner();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    int[] validIDs = {3, 4};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

    m_xController.setTolerance(0.05);
    m_yController.setTolerance(0.05);
    m_rotationController.setTolerance(0.05);

    m_swerveController =
        Choreo.choreoSwerveController(m_xController, m_yController, m_rotationController);

    for (int i = 0; i < 4; i++) {
      var module = getModule(i);
      m_driveSysIdRoutines[i] = createSysIdRoutine(module.getDriveMotor());
      m_steerSysIdRoutines[i] = createSysIdRoutine(module.getSteerMotor());
    }

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

  private SysIdRoutine createSysIdRoutine(TalonFX motor) {
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> motor.setVoltage(volts.in(Volts)),
            log ->
                log.motor("swerveMotor" + motor.hashCode())
                    .voltage(
                        appliedVoltage.mut_replace(
                            motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(angle.mut_replace(motor.getPosition().getValue(), Rotations))
                    .angularVelocity(
                        velocity.mut_replace(
                            (motor.getVelocity().getValue() / 2048.0 * 10), RotationsPerSecond)),
            this));
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

    var currentcommand = getCurrentCommand();
    if (currentcommand != null) {
      m_activeCommand.set(currentcommand.toString());
      m_activeCommandFinished.set(currentcommand.isFinished());
    }

    if (DriverStation.getAlliance().isPresent()) {
      var doRejectUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          "limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2;
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      } else {
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      }
      if (Math.abs(m_pigeon2.getRate())
          > 720) // if our angular velocity is greater than 720 degrees per second, ignore
      // vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2 != null) {
        m_limelightPose.set(mt2.pose);
        if (mt2.tagCount == 0) {
          doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
          m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
          m_odometry.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
      }
    }
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

  public Command sysIdQuasistaticDrive(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutines[moduleIndex].quasistatic(direction);
  }

  /**
   * Creates a command for a dynamic drive SysId routine for the specified module.
   *
   * @param moduleIndex The index of the module.
   * @param direction The direction of the SysId routine.
   * @return A command for the dynamic drive SysId routine.
   */
  public Command sysIdDynamicDrive(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutines[moduleIndex].dynamic(direction);
  }

  /**
   * Creates a command for a quasistatic steer SysId routine for the specified module.
   *
   * @param moduleIndex The index of the module.
   * @param direction The direction of the SysId routine.
   * @return A command for the quasistatic steer SysId routine.
   */
  public Command sysIdQuasistaticSteer(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutines[moduleIndex].quasistatic(direction);
  }

  /**
   * Creates a command for a dynamic steer SysId routine for the specified module.
   *
   * @param moduleIndex The index of the module.
   * @param direction The direction of the SysId routine.
   * @return A command for the dynamic steer SysId routine.
   */
  public Command sysIdDynamicSteer(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutines[moduleIndex].dynamic(direction);
  }

  /**
   * Gets a list of SysId commands for all modules.
   *
   * @return A list of SysId commands.
   */
  public List<Function<SysIdRoutine.Direction, Command>> getSysIdCommands() {
    List<Function<SysIdRoutine.Direction, Command>> sysidCommands = new ArrayList<>();

    for (int i = 0; i < 4; i++) {
      int moduleIndex = i;
      sysidCommands.add(direction -> sysIdDynamicDrive(moduleIndex, direction));
      sysidCommands.add(direction -> sysIdQuasistaticDrive(moduleIndex, direction));
      sysidCommands.add(direction -> sysIdDynamicSteer(moduleIndex, direction));
      sysidCommands.add(direction -> sysIdQuasistaticSteer(moduleIndex, direction));
    }

    return sysidCommands;
  }

  public Command getAutoPath(String pathName) {
    var initPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
    m_odometry.resetPosition(initPose.getRotation(), m_modulePositions, initPose);
    // return new PathPlannerAuto(pathName);
    return AutoBuilder.buildAuto(pathName);
  }
}
