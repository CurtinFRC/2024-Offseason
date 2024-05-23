package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

enum Setpoint {
  kAmpAngle,
  kIntakeAngle,
  kSpeakerAngle,
  kStowed
}

/** Our Arm Subsystem */
public class Arm extends SubsystemBase {
  private PIDController m_pid;
  private CANSparkMax m_primaryMotor;
  private RelativeEncoder m_encoder;
  private ArmFeedforward m_feedforward;

  /**
   * Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
   *
   * @param primaryMotor The primary motor that controls the arm.
   * @param followerMotor The motor that follows the primary motor.
   */
  public Arm(CANSparkMax primaryMotor) {
    m_primaryMotor = primaryMotor;

    m_encoder = m_primaryMotor.getEncoder();
    m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
    m_pid.setTolerance(0.2, 0.5);
    // m_feedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  /** Achieves and maintains speed for the primary motor. */
  private Command achievePosition(double position) {
    return Commands.run(
        () -> m_primaryMotor.setVoltage(m_feedforward.calculate(position, (5676/250)-5) + m_pid.calculate(m_encoder.getPosition(), position)));
  }

  /**
   * Moves the arm to the specified position then stops.
   *
   * @param position The desired position.
   * @return a {@link Command} to get to the desired position.
   */
  private Command moveToPosition(double position) {
    return achievePosition(position).until(m_pid::atSetpoint);
  }

  /**
   * Holds the arm at the current position setpoint.
   *
   * @return A {@link Command} to hold the position at the setpoint.
   */
  public Command maintain() {
    return achievePosition(m_pid.getSetpoint());
  }

  /**
   * Checks if the Arm is at its setpoint and the loop is stable.
   *
   * @return A {@link Trigger} from the result.
   */
  public Trigger atSetpoint() {
    return new Trigger(() -> m_pid.atSetpoint());
  }

  /**
   * Allows for manual control of the arm using a joystick.
   *
   * @param speed The speed from the joystick input.
   * @return A {@link Command} to control the arm manually.
   */
  public Command manualControl(DoubleSupplier speed) {
    return Commands.run(() -> m_primaryMotor.set(speed.getAsDouble()));
  }

  /*
   * Makes the arm go to a setpoint from the {@link Setpoint} enum
   *
   * @param setpoint The setpoint to go to, a {@link Setpoint}
   * @return A {@link Command} to go to the setpoint
   */
  public Command goToSetpoint(Setpoint setpoint) {
    double position = 0;
    switch (setpoint) {
      case kAmpAngle:
      {
        position = -2.17;
        break;
      }

      case kIntakeAngle:
      {
        position = -0.48;
        break;
      }

      case kSpeakerAngle:
      {
        position = -0.82;
        break;
      }

      case kStowed:
      {
        position = -0.52;
        break;
      }
    }
    
    return moveToPosition(position);
  }
}
