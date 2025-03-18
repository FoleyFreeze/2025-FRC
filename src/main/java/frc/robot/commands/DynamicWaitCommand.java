// Copied from WPI WaitCommand but takes a DoubleSupplier for duration

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class DynamicWaitCommand extends Command {
    /** The timer used for waiting. */
    protected Timer m_timer = new Timer();

    private DoubleSupplier m_duration;
    /**
     * Creates a new WaitCommand. This command will do nothing, and end after the specified
     * duration.
     *
     * @param seconds the time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public DynamicWaitCommand(DoubleSupplier seconds) {
        m_duration = seconds;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration.getAsDouble());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
