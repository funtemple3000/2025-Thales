package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class elevator_PS extends ProfiledPIDSubsystem {
  private final TalonFX m_motor_right = new TalonFX(21);
  private final TalonFX m_motor_left = new TalonFX(22);
  private final TalonFX m_encoder = m_motor_right;

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          0, 0, 0, 0);

  @SuppressWarnings("removal")
  public elevator_PS() {
    super(
        new ProfiledPIDController(
            3.5,
            2.0,
            0,
            new TrapezoidProfile.Constraints(
              19,
                25)),
        0);
    setGoal(0.0);
    m_encoder.setPosition(0.0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    m_motor_right.setVoltage(output + feedforward);
    m_motor_left.setVoltage(-1 * (output + feedforward));
  }

  @Override
  public double getMeasurement() {
    System.out.println(m_encoder.getPosition().getValueAsDouble());
    return (m_encoder.getPosition().getValueAsDouble());
  }
}
