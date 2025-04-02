package frc.robot.subsystems;

// Math and PID Imports
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class elevator extends SubsystemBase{
    public TalonFX GB_R = new TalonFX(21);
    public TalonFX GB_L = new TalonFX(22);
    public TalonFX m_encoder = GB_L;

    private TrapezoidProfile.State m_goal;

    private ProfiledPIDController m_controller;
    private TrapezoidProfile.Constraints m_Constraints;

    private final ElevatorFeedforward m_elevatorFF =
    new ElevatorFeedforward(0, 0.0, 0, 0);
    // Substitute values, exchange when available

    public elevator() {
        m_Constraints = new TrapezoidProfile.Constraints(
            3,
            9);
        m_controller = new ProfiledPIDController(
                3,
                0,
                0,
                m_Constraints,
            0.02);
        m_goal = new TrapezoidProfile.State(0.0, 0);
        m_controller.setGoal(m_goal);
    }

    public void setGoal(double goal){
        m_goal = new TrapezoidProfile.State(goal, 0);
        m_controller.setGoal(m_goal);
    }

    public void useOutput(double output) {
    
        double feedforward = m_elevatorFF.calculate(this.m_goal.position, this.m_goal.velocity);
    
        GB_R.setVoltage(-1 * (output + feedforward));
        GB_L.setVoltage((output + feedforward));
    }

    @Override
    public void periodic(){
        useOutput(getPIDout(getMeasurement()));
        System.out.println(m_controller.atGoal());
        boolean atgoal = m_controller.atGoal();
        if(atgoal){stop();}
        System.out.println(m_controller.getPositionError());
    }

    public double getPIDout(double measurement){
        return m_controller.calculate(measurement-m_goal.position);
    }

    public void stop(){
        GB_L.set(0);
        GB_R.set(0);
    }
    
    public double getMeasurement() {
        return (m_encoder.getDifferentialAveragePosition().getValueAsDouble());
    }
    
}
