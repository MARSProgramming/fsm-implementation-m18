package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Wrapper for SequentialCommandGroup so it shows up with a different name on Shuffleboard.
 */
public class ResetSwervePoseCommand extends SequentialCommandGroup {
    public ResetSwervePoseCommand(Command... commands) {
        super(commands);
    }
}