package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder;
import edu.wpi.first.wpilibj.Timer;

public class autodispenseMax extends Command{
    private feeder dispenser;
    private Timer timer;

    public autodispenseMax(Timer t, feeder d){
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
        dispenser.gimmemorpowa();
        // changed dispenser use case
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
