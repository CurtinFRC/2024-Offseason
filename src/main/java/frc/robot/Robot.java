// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autos.OneNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private CommandXboxController m_driver = new CommandXboxController(Constants.driverport);

  private Arm m_arm = new Arm();
  private Shooter m_shooter = new Shooter();
  private Climber m_climber = new Climber();
  private Intake m_intake = new Intake();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private SendableChooser<Auto> m_chooser = new SendableChooser<>();
  private Command m_autonomousCommand;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.DrivebaseMaxSpeed * 0.1)
          .withRotationalDeadband(Constants.DrivebaseMaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(Constants.DrivebaseMaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driver.getLeftY() * Constants.DrivebaseMaxSpeed)
                    .withVelocityY(-m_driver.getLeftX() * Constants.DrivebaseMaxSpeed)
                    .withRotationalRate(
                        -m_driver.getRightX() * Constants.DrivebaseMaxAngularRate)));

    m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
  }

  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    drivetrain.registerTelemetry(logger::telemeterize);

    CommandScheduler.getInstance().registerSubsystem(m_arm);
    CommandScheduler.getInstance().registerSubsystem(m_shooter);
    CommandScheduler.getInstance().registerSubsystem(m_climber);
    CommandScheduler.getInstance().registerSubsystem(m_intake);

    m_chooser.addOption("One Note", new OneNote(m_shooter, m_intake));
    m_chooser.setDefaultOption("One Note", new OneNote(m_shooter, m_intake));
    SmartDashboard.putData(m_chooser);

    configureBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private Command getAutonomousCommand() {
    Auto auto = m_chooser.getSelected();
    auto.configureBindings();
    return auto.followTrajectory();
  }
}
