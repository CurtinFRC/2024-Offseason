package frc.robot;

import org.ejml.equation.Variable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController m_driver;
  private Shooter m_shooter;
  private Climber m_climber;
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Command getAutonomousCommand() {
    
  }

  private void configureBindings() {}

  @SuppressWarnings("removal")
  public Robot() {
    m_driver = new CommandXboxController(Constants.driverport);

    m_shooter =
        new Shooter(
            new CANSparkMax(Constants.shooterPort, CANSparkMaxLowLevel.MotorType.kBrushless));
    CommandScheduler.getInstance().registerSubsystem(m_shooter);

    m_climber =
        new Climber(
            new CANSparkMax(Constants.climberPort, CANSparkMaxLowLevel.MotorType.kBrushless));
    CommandScheduler.getInstance().registerSubsystem(m_climber);
    m_chooser.setDefaultOption("Simple Auto", m_shooter.spinup(1));
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
