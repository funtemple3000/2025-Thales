package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator_PS;
import edu.wpi.first.wpilibj.Timer;

public class autoL3 extends Command {
    private elevator_PS m_elevator;
    private Timer timer;

    public autoL3(elevator_PS L){
        this.m_elevator = L;
        this.timer = new Timer();
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    @SuppressWarnings("removal")
    public void execute(){
        m_elevator.setGoal(-26);
    }

    @Override
    public void end(boolean interrupted){
        m_elevator.eStop();
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(2.2);
    }
}
