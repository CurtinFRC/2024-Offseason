// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
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
import frc.robot.autos.Centre2541;
import frc.robot.autos.Centre26541;
import frc.robot.autos.WompWompKieran;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.HashMap;

public class Robot extends TimedRobot {
  private final CommandXboxController m_driver = new CommandXboxController(Constants.driverport);

  private final Arm m_arm = new Arm();
  private final Shooter m_shooter = new Shooter();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;

  private final SendableChooser<Auto> m_chooser = new SendableChooser<>();
  private Command m_autonomousCommand;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.DrivebaseMaxSpeed * 0.1)
          .withRotationalDeadband(Constants.DrivebaseMaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(Constants.DrivebaseMaxSpeed);

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driver.getLeftY() * Constants.DrivebaseMaxSpeed)
                    .withVelocityY(-m_driver.getLeftX() * Constants.DrivebaseMaxSpeed)
                    .withRotationalRate(
                        -m_driver.getRightX() * Constants.DrivebaseMaxAngularRate)));

    m_driver.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driver.x().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));
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

    m_drivetrain.registerTelemetry(logger::telemeterize);

    CommandScheduler.getInstance().registerSubsystem(m_arm);
    CommandScheduler.getInstance().registerSubsystem(m_shooter);
    CommandScheduler.getInstance().registerSubsystem(m_climber);
    CommandScheduler.getInstance().registerSubsystem(m_intake);

    m_chooser.addOption("Centre2_5_4_1 Blue", new Centre2541(m_drivetrain, false));
    m_chooser.addOption("Centre2_5_4_1 Red", new Centre2541(m_drivetrain, true));
    m_chooser.addOption("Centre2_6_5_4_1 Red", new Centre26541(m_drivetrain, true));
    m_chooser.addOption("Centre2_6_5_4_1 Blue", new Centre26541(m_drivetrain, false));
    m_chooser.addOption("WompWompKieran Blue", new WompWompKieran(m_drivetrain, false));
    m_chooser.addOption("WompWompKieran Red", new WompWompKieran(m_drivetrain, true));
    m_chooser.setDefaultOption("WompWompKieran Blue", new WompWompKieran(m_drivetrain, false));
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
