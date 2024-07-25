// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandRobot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.Centre2541;
import frc.robot.autos.Centre26541;
import frc.robot.autos.WompWompKieran;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends CommandRobot {
  private final CommandXboxController m_driver = new CommandXboxController(Constants.driverport);

  private final Arm m_arm = new Arm();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.DrivebaseMaxSpeed * 0.1)
          .withRotationalDeadband(Constants.DrivebaseMaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry m_logger = new Telemetry();

  private void configureBindings() {
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

    m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_driver.x().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));
    m_driver.leftTrigger().onTrue(m_arm.goToSetpoint(m_drivetrain.getState().Pose).andThen(m_shooter.shootFromFar()));
  }

  public Robot() {
    HashMap<Integer, String> aliases = new HashMap<>();
    aliases.put(31, "Shooter");
    aliases.put(32, "Climber");
    aliases.put(35, "Intake");
    aliases.put(21, "Arm Lead");
    aliases.put(26, "Arm Follower");
    URCL.start(aliases, false);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    SignalLogger.setPath(DataLogManager.getLogDir());
    SignalLogger.start();
    m_drivetrain.registerTelemetry(m_logger::telemeterize);

    m_scheduler.registerSubsystem(m_arm);
    m_scheduler.registerSubsystem(m_shooter);
    m_scheduler.registerSubsystem(m_climber);
    m_scheduler.registerSubsystem(m_intake);

    m_autoChooser.addOption(
        "Centre2_5_4_1 Blue", new Centre2541(m_drivetrain, false).followTrajectory());
    m_autoChooser.addOption(
        "Centre2_5_4_1 Red", new Centre2541(m_drivetrain, true).followTrajectory());
    m_autoChooser.addOption(
        "Centre2_6_5_4_1 Red", new Centre26541(m_drivetrain, true).followTrajectory());
    m_autoChooser.addOption(
        "Centre2_6_5_4_1 Blue", new Centre26541(m_drivetrain, false).followTrajectory());
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
                        Utils.deadzone(-m_driver.getLeftY() * Constants.DrivebaseMaxSpeed))
                    .withVelocityY(
                        Utils.deadzone(-m_driver.getLeftX() * Constants.DrivebaseMaxSpeed))
                    .withRotationalRate(
                        Utils.deadzone(
                            -m_driver.getRightX() * Constants.DrivebaseMaxAngularRate))));

    m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_driver.x().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));
  }
}
