package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DataLogManager; 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

/** Our Crescendo climber Subsystem */
public class Climber extends SubsystemBase {
  private PIDController m_pid;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
 private DataLog log = DataLogManager.getLog();
 private DoubleLogEntry climber_encoder_vel = new DoubleLogEntry(log, "/climber/encoder/velocity");
  private DoubleLogEntry climber_encoder_pos = new DoubleLogEntry(log, "/climber/encoder/position");
  /**
   * Creates a new {@link Climber} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param motor The motor that the climber controls.
   */
  public Climber(CANSparkMax motor) {
    m_motor = motor;
    m_encoder = m_motor.getEncoder();
    climber_encoder_vel.append(m_encoder.getVelocity());
    climber_encoder_pos.append(m_encoder.getPosition());
    m_pid = new PIDController(Constants.climberP, Constants.climberI, Constants.climberD);
    m_pid.setTolerance(0.2, 0.5);
  }

  /**
   * Climb the robot!
   *
   * @return A {@link Command} to climb the robot.
   */
  public Command climb() {
    return Commands.run(
        () ->
            m_motor.setVoltage(
                m_pid.calculate(Units.rotationsToRadians(m_encoder.getPosition() * -1), -3.14159)));
  }
  
}
