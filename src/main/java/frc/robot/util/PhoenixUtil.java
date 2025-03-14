// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class PhoenixUtil {
    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        int i = 0;
        for (; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK()) break;
        }
        if (i == maxAttempts) System.out.println("FAILED TO APPLY CTRE COMMAND!!!!!!!!!!!!!!");
        if (i > 0) System.out.println("CTRE init took " + i + " trys");
    }

    public static void tryUntilOkRev(int maxAttempts, Supplier<REVLibError> command) {
        int i = 0;
        for (; i < maxAttempts; i++) {
            var error = command.get();
            if (error == REVLibError.kOk) break;
        }
        if (i == maxAttempts) System.out.println("FAILED TO APPLY REV COMMAND!!!!!!!!!!!!!!");
        if (i > 0) System.out.println("Rev init took " + i + " trys");
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometryTimeStamps =
                new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometryTimeStamps.length; i++) {
            odometryTimeStamps[i] =
                    Timer.getFPGATimestamp()
                            - 0.02
                            + i * SimulatedArena.getSimulationDt().in(Seconds);
        }

        return odometryTimeStamps;
    }
}
