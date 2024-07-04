// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.OneNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController m_driver;
  private CommandXboxController m_codriver;
  private Arm m_arm;
  private Shooter m_shooter;
  private Climber m_climber;
  private SendableChooser<Auto> m_chooser = new SendableChooser<>();
  private Intake m_intake;

  private Command getAutonomousCommand() {
    Auto auto = m_chooser.getSelected();
    auto.configureBindings();
    return auto.followTrajectory();
  }

  private double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with
                    // negative Y (forward)
                    .withVelocityY(
                        -m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driver.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driver
        .b()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    m_intake.setDefaultCommand(m_intake.spinUntilBeamBreak(20));     

    m_codriver.a().onTrue(m_arm.goToSetpoint(Arm.Setpoint.kAmp));
    m_codriver.b().onTrue(m_arm.goToSetpoint(Arm.Setpoint.kIntake));
    m_codriver.x().onTrue(m_arm.goToSetpoint(Arm.Setpoint.kSpeaker));
    m_codriver.y().onTrue(m_arm.goToSetpoint(Arm.Setpoint.kStowed));

    m_codriver.leftBumper().onTrue(m_shooter.spinup(100));
    m_codriver.rightBumper().onTrue(m_shooter.stop());

    m_codriver.rightTrigger().onTrue(m_shooter.shoot());

    m_codriver.povUp().onTrue(m_climber.climb());

    // if (m_codriver.getHID().getPOV()==0);


  }

  @SuppressWarnings("removal")
  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_driver = new CommandXboxController(Constants.driverport);
    m_codriver = new CommandXboxController(Constants.codriverport);

    var armLead = new CANSparkMax(Constants.armLeadPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    var armFollower =
        new CANSparkMax(Constants.armFollowerPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    armFollower.follow(armLead);
    m_arm = new Arm(armLead);
    CommandScheduler.getInstance().registerSubsystem(m_arm);

    m_shooter =
        new Shooter(
            new CANSparkMax(Constants.shooterPort, CANSparkMaxLowLevel.MotorType.kBrushless),
            new CANSparkMax(Constants.indexerPort, CANSparkMaxLowLevel.MotorType.kBrushless));
    CommandScheduler.getInstance().registerSubsystem(m_shooter);

    m_climber =
        new Climber(
            new CANSparkMax(Constants.climberPort, CANSparkMaxLowLevel.MotorType.kBrushless));
    CommandScheduler.getInstance().registerSubsystem(m_climber);

    m_intake =
        new Intake(Constants.intakebeambreak, new CANSparkMax(Constants.intakePort, CANSparkMaxLowLevel.MotorType.kBrushless));
    CommandScheduler.getInstance().registerSubsystem(m_intake);
    
    final EventLoop m_loop = new EventLoop();

    BooleanEvent UpPOV =
        new BooleanEvent(m_loop, () -> m_codriver.getHID().getPOV()==0)
            // debounce for more stability
            .debounce(0.2); 
    Trigger exampleTrigger = new Trigger(m_loop, () -> m_codriver.getHID().getPOV()==0);
    
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
}
