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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier;
  private double m_lastSimTime;
  private final PIDController m_xController = new PIDController(10, 0, 0.5);
  private final PIDController m_yController = new PIDController(10, 0, 0.5);
  private final PIDController m_rotationController = new PIDController(7, 0, 0.35);
  private ChoreoControlFunction m_swerveController;

  /* Orchestra classes */
  private final Orchestra m_orchestra = new Orchestra();
  private final ArrayList<String> m_songs = new ArrayList<>();
  private static final AudioConfigs m_audioConfig =
      new AudioConfigs().withAllowMusicDurDisable(true);

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective;

  /* SysId Routines for each motor */
  private final SysIdRoutine[] m_driveSysIdRoutines = new SysIdRoutine[4];
  private final SysIdRoutine[] m_steerSysIdRoutines = new SysIdRoutine[4];

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    init();
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    init();
  }

  private void init() {
    if (Utils.isSimulation()) {
      startSimThread();
    }

    m_xController.setTolerance(0.005);
    m_yController.setTolerance(0.005);
    m_rotationController.setTolerance(0.005);

    m_swerveController =
        Choreo.choreoSwerveController(m_xController, m_yController, m_rotationController);

    for (int i = 0; i < 4; i++) {
      var module = getModule(i);
      m_driveSysIdRoutines[i] = createSysIdRoutine(module.getDriveMotor());
      m_steerSysIdRoutines[i] = createSysIdRoutine(module.getSteerMotor());
    }
  }

  private SysIdRoutine createSysIdRoutine(TalonFX motor) {
    // RelativeEncoder encoder = motor.clearStickyFault_ReverseHardLimit();
    MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

    return new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> motor.setVoltage(volts.in(Volts)),
            log ->
                log.motor("swerveMotor")
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

  public void addMusic(String... songs) {
    m_songs.addAll(Arrays.asList(songs));
  }

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

  public ArrayList<String> getSongs() {
    return m_songs;
  }

  public Orchestra getOrchestra() {
    return m_orchestra;
  }

  @Override
  public void periodic() {
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

  public Command sysIdQuasistaticDrive(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutines[moduleIndex].quasistatic(direction);
  }

  public Command sysIdDynamicDrive(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_driveSysIdRoutines[moduleIndex].dynamic(direction);
  }

  public Command sysIdQuasistaticSteer(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutines[moduleIndex].quasistatic(direction);
  }

  public Command sysIdDynamicSteer(int moduleIndex, SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutines[moduleIndex].dynamic(direction);
  }

  public List<java.util.function.Function<SysIdRoutine.Direction, Command>> getSysIdCommands() {
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
}
