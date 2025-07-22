package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.led.LEDSubsystem;

public class TimedFlashLEDCommand extends Command {
    LEDSubsystem ledSubsystem;
    Superstructure superstructure;
    private edu.wpi.first.wpilibj.Timer timer;
    private LEDSubsystem.WantedState wantedState;
    private double timeToRun;

    public TimedFlashLEDCommand(
            Superstructure superstructure,
            LEDSubsystem ledSubsystem,
            LEDSubsystem.WantedState wantedState,
            double timeToRun) {
        this.superstructure = superstructure;
        this.ledSubsystem = ledSubsystem;
        timer = new edu.wpi.first.wpilibj.Timer();
        this.wantedState = wantedState;
        this.timeToRun = timeToRun;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
        ledSubsystem.setWantedAction(wantedState);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeToRun)) {
            ledSubsystem.setWantedAction(LEDSubsystem.WantedState.DISPLAY_OFF);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}