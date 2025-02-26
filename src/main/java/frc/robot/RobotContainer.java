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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.controls.BotState;
import frc.robot.subsystems.controls.ControlBoard;
import frc.robot.subsystems.controls.Flysky;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.vision.Vision;
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
    public final Vision vision;

    // Controller
    public final Flysky flysky = new Flysky();
    public final ControlBoard controlBoard = new ControlBoard(this);
    public final BotState state = new BotState();

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
        wrist = Wrist.create(this);
        arm = Arm.create();
        elevator = Elevator.create();
        hand = Hand.create();
        climb = Climb.create();
        vision = Vision.create(this);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        autoChooser.addDefaultOption("Selectable", null);

        autoChooser.addOption(
                "JustDrive",
                new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(0.1, 0, 0)), drive));

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
                        this,
                        () -> -flysky.getLeftY(),
                        () -> -flysky.getLeftX(),
                        () -> -flysky.getRightX()));

        elevator.setDefaultCommand(ComplexCommands.homeLogic());

        climb.setDefaultCommand(climb.setClimbVoltage(0));

        // Reset gyro to 0° when B button is pressed
        flysky.upLTRIM.onTrue(DriveCommands.zeroDrive(this).ignoringDisable(true));

        // rezero superstructure
        flysky.upRTRIM.onTrue(ComplexCommands.zeroSuperstructure().ignoringDisable(true));

        // left trigger commands
        // gather coral camera
        // gather coral nocam
        // gather algae camera
        // gather algae nocam
        // climb down
        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(flysky.topLeftSWA.negate()) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                // .whileTrue(ComplexCommands.visionCoralGather());
                .whileTrue(ComplexCommands.blindGatherCoral());

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(flysky.topLeftSWA.negate()) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.blindGatherCoral());

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(flysky.topLeftSWA) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.visionAlgaeGather());

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(flysky.topLeftSWA) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.gatherAlgae());

        flysky.leftTriggerSWE // gather sw
                .and(controlBoard.climbModeT) // climb sw
                .whileTrue(r.climb.setClimbVoltage(-2));

        // right trigger commands
        // score coral camera
        // score coral nocam
        // score algae camera
        // score algae nocam
        // climb up

        flysky.rightTriggerSWG // gather sw
                .and(flysky.topLeftSWA.negate()) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.visionCoralScore());

        flysky.rightTriggerSWG // gather sw
                .and(flysky.topLeftSWA.negate()) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.blindCoralScore());

        flysky.rightTriggerSWG // gather sw
                .and(flysky.topLeftSWA) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.visionAlgaeScore());

        flysky.rightTriggerSWG // gather sw
                .and(flysky.topLeftSWA) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(ComplexCommands.scoreAlgaeProc());

        flysky.rightTriggerSWG // gather sw
                .and(controlBoard.climbModeT) // climb sw
                .whileTrue(r.climb.setClimbVoltage(6));

        // stop button
        flysky.topRightMomentSWC.onTrue(ComplexCommands.stopSuperstructure().ignoringDisable(true));
    }

    public void robotPeriodic() {
        controlBoard.periodic();

        String s =
                String.format(
                        "%.1f,%.0f,%.0f",
                        r.elevator.getHeight().in(Inches),
                        r.arm.getAngle().in(Degrees),
                        r.wrist.getAngleRads().in(Degrees));
        SmartDashboard.putString("SuperPosition", s);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
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
        mechArm.setAngle(arm.getAngle().in(Degrees));
        mechWrist.setAngle(wrist.getAngleRads().in(Degrees));

        Logger.recordOutput("Mechanism", mechBase);
    }
}
