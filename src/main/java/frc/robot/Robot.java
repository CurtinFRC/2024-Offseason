// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandRobot;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.WompWompKieran;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.Setpoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sysid;

import java.util.HashMap;

public class Robot extends CommandRobot {
  private final CommandXboxController m_driver = new CommandXboxController(Constants.driverport);
  private final CommandXboxController m_codriver =
      new CommandXboxController(Constants.codriverport);
  private final CommandXboxController m_test = new CommandXboxController(Constants.testport);

  private final Arm m_arm = new Arm();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Index m_index = new Index();
  private final Sysid m_sysid = new Sysid();
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

  public Robot() {
    HashMap<Integer, String> urclAliases = new HashMap<>();
    urclAliases.put(Constants.shooterPort, "Shooter");
    urclAliases.put(Constants.climberPort, "Climber");
    urclAliases.put(Constants.intakePort, "Intake");
    urclAliases.put(Constants.indexerPort, "Index");
    urclAliases.put(Constants.armLeadPort, "Arm Lead");
    urclAliases.put(Constants.armFollowerPort, "Arm Follower");

    DataLogManager.start();
    URCL.start(urclAliases, false);
    DriverStation.startDataLog(DataLogManager.getLog());
    SignalLogger.setPath(DataLogManager.getLogDir());
    SignalLogger.start();
    // m_sysid.add(m_shooter::sysIdDynamic);
    // m_sysid.add(m_shooter::sysIdQuasistatic);

    // m_sysid.add(m_arm::sysIdDynamic);
    // m_sysid.add(m_arm::sysIdQuasistatic);

    // m_sysid.add(m_climber::sysIdDynamic);
    // m_sysid.add(m_climber::sysIdQuasistatic);

    m_sysid.add(m_intake::sysIdDynamic);
    m_sysid.add(m_intake::sysIdQuasistatic);

    // m_sysid.addAll(m_drivetrain.getSysIdCommands());

    m_drivetrain.registerTelemetry(m_logger::telemeterize);

    m_scheduler.registerSubsystem(m_arm);
    m_scheduler.registerSubsystem(m_shooter);
    m_scheduler.registerSubsystem(m_climber);
    m_scheduler.registerSubsystem(m_intake);
    m_scheduler.registerSubsystem(m_index);

    m_autoChooser.addOption(
        "WompWompKieran Blue", new WompWompKieran(m_drivetrain, false).followTrajectory());
    m_autoChooser.addOption(
        "WompWompKieran Red", new WompWompKieran(m_drivetrain, true).followTrajectory());
    m_autoChooser.setDefaultOption(
        "WompWompKieran Blue", new WompWompKieran(m_drivetrain, false).followTrajectory());
    SmartDashboard.putData(m_autoChooser);

    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(
                        Utils.deadzone(-m_driver.getLeftY() * Constants.DrivebaseMaxSpeed * 0.5))
                    .withVelocityY(
                        Utils.deadzone(-m_driver.getLeftX() * Constants.DrivebaseMaxSpeed * 0.5))
                    .withRotationalRate(
                        Utils.deadzone(
                            -m_driver.getRightX() * Constants.DrivebaseMaxAngularRate))));
    m_intake.setDefaultCommand(m_superstructure.intake());
    m_shooter.setDefaultCommand(m_shooter.stop());
    m_index.setDefaultCommand(m_index.stop());
    m_arm.setDefaultCommand(m_arm.goToSetpoint(Setpoint.kIntake));

    new Trigger(() -> m_codriver.getLeftY() > 0.05)
        .whileTrue(m_arm.manualControl(m_codriver::getLeftY));

    m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_driver.y().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    m_codriver.a().onTrue(m_climber.climb());
    m_codriver.leftBumper().whileTrue(m_index.shoot());
    m_codriver.rightBumper().whileTrue(m_shooter.spinup(500).andThen(m_shooter.maintain()));
    m_codriver
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_arm.goToSetpoint(Setpoint.kSpeaker).andThen(m_arm.maintain()),
                m_shooter.spinup(500).andThen(m_shooter.maintain())));
    m_codriver
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                m_arm.goToSetpoint(Setpoint.kAmp).andThen(m_arm.maintain()),
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
    m_codriver.b().whileTrue(m_superstructure.outake());

    m_driver.b().onTrue(m_sysid.getAllCommands());
  }
}
