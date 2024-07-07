// Copyright (c) 2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends ProfiledPIDSubsystem {

    public enum Setpoint {
        kAmp,
        kIntake,
        kSpeaker,
        kStowed
      }

  private final CANSparkMax m_motor = new CANSparkMax(Constants.armLeadPort, MotorType.kBrushless);
  private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(Constants.armEncoderPort);
  private final ArmFeedforward 
  m_feedforward =
      new ArmFeedforward(
          Constants.armS, Constants.armG,
          Constants.armV, Constants.armA);

public Arm() {
    super(
        new ProfiledPIDController(
            Constants.armP,
            Constants.armI,
            Constants.armD,
            new TrapezoidProfile.Constraints(
                Constants.armMaxVelRadPerSec,
                Constants.armMaxAccelRadPerSec)),
        0);
   
    // Start arm at rest in neutral position
    setGoal(Constants.armOffsetRad);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override 
  public double getMeasurement() {
    return m_encoder.getDistance() + Constants.armOffsetRad;
  }

  public void goToSetpoint(Setpoint setpoint) {
    double position = 0;
    // log_setpoint.append(setpoint.name());

    switch (setpoint) {
      case kAmp:
        position = 5.34;
        break;

      case kIntake:
        position = 3.7;
        break;

      case kSpeaker:
        position = 3.7;
        break;

      case kStowed:
        position = 3.7;
        break;
    }

    setGoal(position);
  }

}

/** Our Arm Subsystem */
// public class Arm extends SubsystemBase {
//   public enum Setpoint {
//     kAmp,
//     kIntake,
//     kSpeaker,
//     kStowed
//   }

//   private PIDController m_pid;
//   private CANSparkMax m_primaryMotor;
//   private CANSparkMax m_followerMotor;
//   private DutyCycleEncoder m_encoder;
//   private ArmFeedforward m_feedforward;
//   private DataLog m_log = DataLogManager.getLog();
//   private DoubleLogEntry log_pid_output = new DoubleLogEntry(m_log, "/arm/pid/output");
//   private DoubleLogEntry log_pid_setpoint = new DoubleLogEntry(m_log, "/arm/pid/setpoint");
//   private DoubleLogEntry log_ff_position_setpoint =
//       new DoubleLogEntry(m_log, "/arm/ff/position_setpoint");
//   private DoubleLogEntry log_ff_velocity_setpoint =
//       new DoubleLogEntry(m_log, "/arm/ff/velocity_setpoint");
//   private DoubleLogEntry log_ff_output = new DoubleLogEntry(m_log, "/arm/ff/output");
//   private StringLogEntry log_setpoint = new StringLogEntry(m_log, "/arm/setpoint");


//   /**
//    * Creates a new {@link Arm} {@link edu.wpi.first.wpilibj2.command.Subsystem}.
//    *
//    * @param primaryMotor The primary motor that controls the arm.
//    */
//   public Arm() {
//     m_primaryMotor = new CANSparkMax(Constants.armLeadPort, MotorType.kBrushless);
//     m_followerMotor = new CANSparkMax(Constants.armFollowerPort, MotorType.kBrushless);
//     m_followerMotor.follow(m_primaryMotor);

//     m_encoder = new DutyCycleEncoder(Constants.armEncoderPort);
//     m_pid = new PIDController(Constants.armP, Constants.armI, Constants.armD);
//     m_pid.setTolerance(0.2);
//     m_feedforward =
//         new ArmFeedforward(Constants.armS, Constants.armG, Constants.armV, Constants.armA);
//   }

//   /** Achieves and maintains speed for the primary motor. */
//   private Command achievePosition(double position) {
//     return Commands.run(
//         () -> {
//           var pid_output = m_pid.calculate(m_encoder.getAbsolutePosition() * 2 * 3.14, position);
//           log_pid_output.append(pid_output);
//           log_pid_setpoint.append(m_pid.getSetpoint());
//           var ff_output = m_feedforward.calculate(position, (5676 / 250));
//           log_ff_output.append(ff_output);
//           log_ff_position_setpoint.append(position);
//           log_ff_velocity_setpoint.append((5676 / 250));
//           m_primaryMotor.setVoltage(-1 * ff_output + pid_output);
//         });
//   }

//   public Command stop() {
//     return runOnce(() -> m_primaryMotor.set(0));
//   }

//   /**
//    * Moves the arm to the specified position then stops.
//    *
//    * @param position The desired position.
//    * @return a {@link Command} to get to the desired position.
//    */
//   private Command moveToPosition(double position) {
//     return achievePosition(position)
//         .until(() -> m_pid.atSetpoint() && m_encoder.getAbsolutePosition() * 2 * 3.14 == position);
//   }

//   /**
//    * Holds the arm at the current position setpoint.
//    *
//    * @return A {@link Command} to hold the position at the setpoint.
//    */
//   public Command maintain() {
//     return achievePosition(m_pid.getSetpoint());
//   }

//   /**
//    * Checks if the Arm is at its setpoint and the loop is stable.
//    *
//    * @return A {@link Trigger} from the result.
//    */
//   public Trigger atSetpoint() {
//     return new Trigger(() -> m_pid.atSetpoint());
//   }

//   /**
//    * Allows for manual control of the arm using a joystick.
//    *
//    * @param speed The speed from the joystick input.
//    * @return A {@link Command} to control the arm manually.
//    */
//   public Command manualControl(DoubleSupplier speed) {
//     return Commands.run(() -> m_primaryMotor.set(speed.getAsDouble()), this);
//   }

//   /*
//    * Makes the arm go to a setpoint from the {@link Setpoint} enum
//    *
//    * @param setpoint The setpoint to go to, a {@link Setpoint}
//    *
//    * @return A {@link Command} to go to the setpoint
//    */
  // public Command goToSetpoint(Setpoint setpoint) {
  //   double position = 0;
  //   log_setpoint.append(setpoint.name());

  //   switch (setpoint) {
  //     case kAmp:
  //       position = 5.34;
  //       break;

  //     case kIntake:
  //       position = 3.7;
  //       break;

  //     case kSpeaker:
  //       position = 3.7;
  //       break;

  //     case kStowed:
  //       position = 3.7;
  //       break;
  //   }

  //   return moveToPosition(position);
  // }
// }
