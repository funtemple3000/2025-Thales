package frc.robot.subsystems;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix6.hardware.TalonFX;


public class feeder {
    private TalonFX motorD;
    public feeder(TalonFX motor1){
        this.motorD = motor1;
    }
    public void dispense(){ motorD.set(-0.1); }
    public void gimmemorpowa(){ motorD.set(-0.15); }
    public void hold(){ motorD.set(0.05); }
    // Above: Idle function to prevent problems
    public void stop(){ motorD.set(0.0); }
}
