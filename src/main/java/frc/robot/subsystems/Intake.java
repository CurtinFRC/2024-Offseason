package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;


public class Intake extends SubsystemBase {
    private PIDController m_pid;
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;
    private DigitalInput m_beamBreakSensor;


    public Intake(int beamBreakChannel, CANSparkMax motor) {
        m_motor = motor;
        m_encoder = m_motor.getEncoder();
        m_pid = new PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD);
        m_pid.setTolerance(0.2, 0.5);
        m_beamBreakSensor = new DigitalInput(beamBreakChannel); // Initialize DigitalInput for beam break sensor
    }


    //sensor
    public Command achieveSpeed(double speed) {
        return Commands.run(
            () ->
                m_motor.setVoltage(
                    m_pid.calculate(Units.rotationsToRadians(m_encoder.getVelocity()), speed)));
    }

    public Command spinup(double speed) {
        return achieveSpeed(speed).until(m_pid::atSetpoint);
    }

    public Trigger atSetpoint() {
        return new Trigger(
            () ->
                m_pid.getSetpoint() == Units.rotationsToRadians(m_encoder.getVelocity())
                    && m_pid.atSetpoint());
    }

    public boolean isBeamBroken() {
        return !m_beamBreakSensor.get(); 
    }

    public Command spinUntilBeamBreak(double speed) {
        return achieveSpeed(speed)
            .until(this::isBeamBroken)
            .andThen(() -> m_motor.stopMotor(), this);
    }

    // stop spinning
    public Command stopIntake() {
        return new RunCommand(() -> m_motor.stopMotor(), this);
    }
    //end sensor
}  
    