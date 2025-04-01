package frc.robot.subsystems;

// Math and PID Imports
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class elevator extends ProfiledPIDSubsystem {
    public TalonFX GB_R = new TalonFX(1);
    public TalonFX GB_L = new TalonFX(1);
    public TalonFX m_encoder = GB_L;

    private final ElevatorFeedforward m_elevatorFF =
    new ElevatorFeedforward(0, 0, 0, 0);
    // Substitute values, exchange when available

    @SuppressWarnings("removal")
    public elevator() {
        super(
            new ProfiledPIDController(
                0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                    0,
                    0)),
            0);
        setGoal(0);
    }
    
    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
    
        double feedforward = m_elevatorFF.calculate(setpoint.position, setpoint.velocity);
    
        GB_R.setVoltage(output + feedforward);
        GB_L.setVoltage(-1 * (output + feedforward));
    }
    
    @Override
    public double getMeasurement() {
        return (m_encoder.getDifferentialAveragePosition().getValueAsDouble());
    }
}
