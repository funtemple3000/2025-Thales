package frc.robot.subsystems;

// Math and PID Imports
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class elevatorDOWN extends ProfiledPIDController {
    public TalonFX GB_R = new TalonFX(1);
    public TalonFX GB_L = new TalonFX(1);

    private final ElevatorFeedforward m_elevatorFF =
    new ElevatorFeedforward(
        pivotUpConstants.kSVolts, pivotUpConstants.kGVolts,
        pivotUpConstants.kVVoltSecondPerRad, pivotUpConstants.kAVoltSecondSquaredPerRad);
}
