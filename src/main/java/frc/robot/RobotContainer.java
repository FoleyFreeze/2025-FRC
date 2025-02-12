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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.controls.ControlBoard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.wrist.Wrist;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Elevator elevator;
    public final Arm arm;
    public final Wrist wrist;
    public final Hand hand;
    public final Climb climb;

    // Controller
    public final CommandXboxController controller = new CommandXboxController(0);
    public final ControlBoard controlBoard = new ControlBoard();

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private static RobotContainer r = null;

    public static RobotContainer getInstance() {
        return r;
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        r = this;
        ComplexCommands.r = this;

        drive = Drive.create();
        wrist = Wrist.create();
        arm = Arm.create();
        elevator = Elevator.create();
        hand = Hand.create();
        climb = Climb.create();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

        // Lock to 0° when A button is held
        /*controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));*/

        // Switch to X pattern when X button is pressed
        // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
                .button(10)
                .onTrue(
                        Commands.runOnce(
                                        Constants.currentMode == Constants.Mode.SIM
                                                ? () ->
                                                        drive.setPose(
                                                                drive.driveSimulation
                                                                        .getSimulatedDriveTrainPose())
                                                : () ->
                                                        drive.setPose(
                                                                new Pose2d(
                                                                        drive.getPose()
                                                                                .getTranslation(),
                                                                        new Rotation2d())),
                                        drive)
                                .andThen(
                                        new InstantCommand(
                                                () ->
                                                        SimulatedArena.getInstance()
                                                                .resetFieldForAuto()))
                                .ignoringDisable(true));

        // rezero superstructure
        controller.button(14).onTrue(ComplexCommands.zeroSuperstructure().ignoringDisable(true));

        controller.axisGreaterThan(3, 0).whileTrue(ComplexCommands.noDriveScore());
        controller
                .axisGreaterThan(2, 0)
                .and(controller.axisGreaterThan(3, 0).negate())
                .whileTrue(ComplexCommands.noDriveGather());
        controller.button(4).onTrue(ComplexCommands.stopSuperstructure().ignoringDisable(true));

        // controller.button(2).onTrue(new InstantCommand(() ->
        // goTo(SuperstructureLocation.LEVEL4)));
        // controller.button(3).onTrue(new InstantCommand(() ->
        // goTo(SuperstructureLocation.INTAKE)));
    }

    public void robotPeriodic() {
        String s =
                String.format(
                        "%.1f,%.0f,%.0f",
                        r.elevator.getHeight().in(Inches),
                        r.arm.getAngleRads().in(Degrees),
                        r.wrist.getAngleRads().in(Degrees));
        SmartDashboard.putString("SuperPosition", s);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: fix that this crashes if you run auton more than once
        return autoChooser.get().andThen(drive::stopWithX, drive);
    }

    public void resetSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        drive.setPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        Logger.recordOutput(
                "FieldSimulation/RobotPosition",
                drive.driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    LoggedMechanism2d mechBase = new LoggedMechanism2d(20, 50);
    LoggedMechanismRoot2d mechRoot = mechBase.getRoot("Root", 10, 0);
    LoggedMechanismLigament2d mechElevator =
            mechRoot.append(new LoggedMechanismLigament2d("Elevator", 0, 90));
    LoggedMechanismLigament2d mechArm =
            mechElevator.append(new LoggedMechanismLigament2d("Arm", .5, 60));
    LoggedMechanismLigament2d mechWrist =
            mechArm.append(new LoggedMechanismLigament2d("Wrist", .3, 60));

    public void updateMechanisms() {
        mechElevator.setLength(elevator.getHeight().in(Meters));
        mechArm.setAngle(arm.getAngleRads().in(Degrees));
        mechWrist.setAngle(wrist.getAngleRads().in(Degrees));

        Logger.recordOutput("Mechanism", mechBase);
    }
}
