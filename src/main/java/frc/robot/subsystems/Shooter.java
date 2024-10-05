// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import revlibsim.SparkMax;

/** Our Crescendo shooter Subsystem */
public class Shooter extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(Constants.shooterPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final DCMotorSim m_simMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.0021896, 0.00070176), DCMotor.getNEO(1), 1.25);

  private final NetworkTable driveStats = NetworkTableInstance.getDefault().getTable("Shooter");
  private final StringPublisher m_activeCommand =
      driveStats.getStringTopic("Active Command").publish();
  private final BooleanPublisher m_activeCommandFinished =
      driveStats.getBooleanTopic("Active Command Finished").publish();

  private final PIDController m_pid =
      new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/shooter/pid/output");
  private final NetworkTable shooterStats = NetworkTableInstance.getDefault().getTable("Shooter");
  private final DoublePublisher m_ntPidError = shooterStats.getDoubleTopic("PID/Error").publish();
  private final DoublePublisher m_ntPidOutput = shooterStats.getDoubleTopic("PID/Output").publish();
  private final DoublePublisher m_ntPidSetpoint =
      shooterStats.getDoubleTopic("PID/Setpoint").publish();
  private final DoublePublisher m_ntRotationalVelocity =
      shooterStats.getDoubleTopic("RotationalVelocity").publish();

  public final Trigger m_atSetpoint = new Trigger(m_pid::atSetpoint);

  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(Constants.shooterS, Constants.shooterV, Constants.shooterA);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                m_motor.setVoltage(volts.in(Volts));
              },
              null,
              this));

  /** Creates a new {@link Shooter} {@link edu.wpi.first.wpilibj2.command.Subsystem}. */
  public Shooter() {}

  /** Achieves and maintains speed. */
  private Command achieveSpeeds(double _speed) {
    var speed = Units.radiansPerSecondToRotationsPerMinute(_speed);
    m_pid.setSetpoint(speed);
    return run(
        () -> {
          var output = m_pid.calculate(-1 * m_encoder.getVelocity(), speed);
          log_pid_output.append(output);
          m_ntPidError.set(m_pid.getVelocityError());
          m_ntPidOutput.set(m_pid.getVelocityError());
          m_ntPidSetpoint.set(m_pid.getSetpoint());
          m_motor.setVoltage(output + m_feedforward.calculate(speed));
        });
  }

  /**
   * Speeds up the shooter until it hits the specified speed then stops.
   *
   * @param speed The desired speed in Radians per second.
   * @return a {@link Command} to get to the desired speed.
   */
  public Command spinup(double speed) {
    return defer(() -> achieveSpeeds(speed).until(m_pid::atSetpoint));
  }

  public Command stop() {
    return runOnce(() -> m_motor.set(0));
  }

  /**
   * Holds the shooter at the current speed setpoint.
   *
   * @return A {@link Command} to hold the speed at the setpoint.
   */
  public Command maintain() {
    return defer(() -> achieveSpeeds(m_pid.getSetpoint()));
  }

  public Command shoot() {
    return spinup(500).andThen(maintain());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command applyVolts(double volts) {
    return run(() -> m_motor.setVoltage(volts));
  }

  @Override
  public void periodic() {
    m_ntRotationalVelocity.set(Units.rotationsPerMinuteToRadiansPerSecond(m_encoder.getVelocity()));

    var currentcommand = getCurrentCommand();
    if (currentcommand != null) {
      m_activeCommand.set(currentcommand.toString());
      m_activeCommandFinished.set(currentcommand.isFinished());
    }
  }

  @Override
  public void simulationPeriodic() {
    m_simMotor.update(0.02);
    m_encoder.setPosition(m_simMotor.getAngularPositionRotations());
  }
}
