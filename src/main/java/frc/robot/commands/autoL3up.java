package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator_PS;

public class autoL3up extends Command {
    private elevator_PS m_elevator;

    public autoL3up(elevator_PS L){
        this.m_elevator = L;
    }

    @Override
    public void initialize(){
    }

    @Override
    @SuppressWarnings("removal")
    public void execute(){
        m_elevator.setGoal(-26);
    }
}
