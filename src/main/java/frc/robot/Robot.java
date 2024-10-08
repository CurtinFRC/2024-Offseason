// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandRobot;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Setpoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.RotationSetpoint;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import java.util.Map;
import java.util.Set;

public class Robot extends CommandRobot {
  private final CommandXboxController m_driver = new CommandXboxController(Constants.driverport);
  private final CommandXboxController m_codriver =
      new CommandXboxController(Constants.codriverport);

  private final Arm m_arm = new Arm();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();
  private final LED m_led = new LED();
  private final Superstructure m_superstructure = new Superstructure(m_shooter, m_intake, m_index);
  private static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.DrivebaseMaxSpeed * 0.1)
          .withRotationalDeadband(Constants.DrivebaseMaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry m_logger = new Telemetry();

  private final Trigger m_codriverX = m_codriver.x();

  double armangle = 0.2;

  public Robot() {
    DataLogManager.start();
    URCL.start(
        Map.of(
            Constants.shooterPort, "Shooter",
            Constants.climberPort, "Climber",
            Constants.intakePort, "Intake",
            Constants.indexerPort, "Index",
            Constants.armLeadPort, "Arm Lead",
            Constants.armFollowerPort, "Arm Follower"),
        false);
    DriverStation.startDataLog(DataLogManager.getLog());
    SignalLogger.setPath(DataLogManager.getLogDir());
    SignalLogger.start();

    m_drivetrain.registerTelemetry(m_logger::telemeterize);

    m_scheduler.registerSubsystem(m_arm);
    m_scheduler.registerSubsystem(m_shooter);
    m_scheduler.registerSubsystem(m_climber);
    m_scheduler.registerSubsystem(m_intake);
    m_scheduler.registerSubsystem(m_index);
    m_scheduler.registerSubsystem(m_led);

    NamedCommands.registerCommand(
        "Shoot",
        Commands.deferredProxy(
            () ->
                m_superstructure
                    .shoot()
                    .withTimeout(2)
                    .andThen(Commands.parallel(m_shooter.stop(), m_index.stop()))));
    NamedCommands.registerCommand(
        "Shoot2",
        m_arm
            .moveToPosition(0.7528)
            .andThen(
                Commands.deferredProxy(
                    () ->
                        Commands.parallel(
                            m_superstructure
                                .shoot()
                                .withTimeout(2)
                                .andThen(Commands.parallel(m_shooter.stop(), m_index.stop())),
                            m_arm.maintain()))));
    NamedCommands.registerCommand(
        "Arm",
        Commands.deferredProxy(() -> m_arm.goToSetpointAuto(Setpoint.kSpeaker).withTimeout(1.2)));
    NamedCommands.registerCommand(
        "Intake", Commands.deferredProxy(() -> m_superstructure.intake()));
    NamedCommands.registerCommand("OTFArm", m_arm.moveToPosition(0.7528).andThen(m_arm.maintain()));
    NamedCommands.registerCommand(
        "Spinup",
        Commands.deferredProxy(() -> m_shooter.spinup(500).andThen(m_shooter.maintain())));
    NamedCommands.registerCommand("Passthrough", Commands.deferredProxy(() -> m_index.shootAuto()));

    m_autoChooser.addOption("Centre1423", m_drivetrain.getAutoPath("Centre1423"));
    m_autoChooser.addOption("Centre1423Blue", m_drivetrain.getAutoPath("Centre1423Blue"));
    m_autoChooser.setDefaultOption("Centre1423", m_drivetrain.getAutoPath("Centre1423"));
    SmartDashboard.putData(m_autoChooser);
    SmartDashboard.putNumber("Arm", armangle);

    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(
                        Utils.deadzone(-m_driver.getLeftY() * Constants.DrivebaseMaxSpeed))
                    .withVelocityY(
                        Utils.deadzone(-m_driver.getLeftX() * Constants.DrivebaseMaxSpeed))
                    .withRotationalRate(
                        Utils.deadzone(
                            -m_driver.getRightX() * Constants.DrivebaseMaxAngularRate))));
    m_intake.setDefaultCommand(m_superstructure.intake());
    m_shooter.setDefaultCommand(m_shooter.stop());
    m_index.setDefaultCommand(m_index.stop());
    m_arm.setDefaultCommand(m_arm.goToSetpoint(Setpoint.kIntake));
    m_led.setDefaultCommand(m_led.hotpink());

    new Trigger(() -> m_codriver.getLeftY() > 0.05)
        .whileTrue(m_arm.manualControl(m_codriver::getLeftY));

    m_codriver
        .povUp()
        .whileTrue(
            Commands.defer(
                () ->
                    m_arm
                        .moveToPosition(SmartDashboard.getNumber("Arm", 0.2))
                        .andThen(m_arm.maintain()),
                Set.of(m_arm)));

    m_shooter.m_atSetpoint.whileTrue(m_led.canShoot());
    m_index.m_hasNote.whileTrue(m_led.hasNote());
    m_index.m_intaking.whileTrue(m_led.intaking());

    m_index.m_intaking.onTrue(
        Commands.parallel(
            m_intake.intake(5).until(m_index.m_hasNote).andThen(m_intake.stop()),
            m_index
                .intake(-3.5)
                .until(m_index.m_hasNote)
                .andThen(new WaitCommand(0.5))
                .andThen(m_index.intake(2))
                .until(m_index.m_hasNote)
                .andThen(m_index.stop())));

    m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_driver.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));
    m_driver
        .leftBumper()
        .onTrue(m_drivetrain.goToSetpoint(RotationSetpoint.kAmp, m_drive, m_scheduler));
    m_driver
        .rightBumper()
        .onTrue(m_drivetrain.goToSetpoint(RotationSetpoint.kSpeaker, m_drive, m_scheduler));

    m_codriver.leftBumper().whileTrue(m_index.shoot());
    m_codriver.rightBumper().whileTrue(m_superstructure.outake());
    m_codriver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_arm
                    .goToSetpoint(Setpoint.kSpeaker, m_codriver.leftTrigger())
                    .andThen(m_arm.maintain()),
                m_shooter.spinup(1500).andThen(m_shooter.maintain())));
    m_codriver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                m_arm
                    .goToSetpoint(Setpoint.kAmp, m_codriver.rightTrigger())
                    .andThen(m_arm.maintain()),
                m_shooter.applyVolts(8)));

    m_codriverX.whileTrue(m_superstructure.intake()).onFalse(m_intake.stop());
    m_scheduler
        .getDefaultButtonLoop()
        .bind(
            () -> {
              if (m_codriver.getHID().getYButton()) {
                m_intake.removeDefaultCommand();
              }
            });
    m_codriver.y().onTrue(m_superstructure.stop());
    m_codriver.b().whileTrue(m_shooter.spinup(1500).andThen(m_shooter.maintain()));
  }
}
