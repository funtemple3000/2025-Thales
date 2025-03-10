package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder;
import edu.wpi.first.wpilibj.Timer;

public class autodispense extends Command{
    private feeder dispenser;
    private Timer timer;

    public autodispense(Timer t, feeder d){
        this.dispenser = d;
        this.timer = t;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        dispenser.dispense();
    }

    @Override
    public void end(boolean interrupted){
        dispenser.stop();
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(1);
    }
}

