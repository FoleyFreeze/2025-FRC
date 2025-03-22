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
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.auton.AutonCommands;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.subsystems.LEDs.LED;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.controls.BotState;
import frc.robot.subsystems.controls.ControlBoard;
import frc.robot.subsystems.controls.Flysky;
import frc.robot.subsystems.cvision.CVision;
import frc.robot.subsystems.cvision.CVisionCals;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.PathCache;
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
    public PathCache pathCache;
    public final CVision cvision;
    public final LED leds;

    // Controller
    public final Flysky flysky = new Flysky();
    public final ControlBoard controlBoard = new ControlBoard(this);
    public final BotState state = new BotState(this);

    // Pathplanner triggers
    public EventTrigger inSlowDrivePhase;
    public DigitalInput dio1 = new DigitalInput(9);
    public Trigger neutralSwitch = new Trigger(() -> !dio1.get() && DriverStation.isDisabled());

    public Trigger isDisabledOrAuto =
            new Trigger(() -> DriverStation.isDisabled() || DriverStation.isAutonomous());

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
        AutonCommands.r = this;

        drive = Drive.create();
        wrist = Wrist.create(this);
        arm = Arm.create();
        elevator = Elevator.create();
        hand = Hand.create();
        climb = Climb.create();
        vision = Vision.create(this);
        cvision = new CVision(r, new CVisionCals());
        leds = new LED(r);

        inSlowDrivePhase = new EventTrigger("InSlowDrivePhase");

        // Set up auto routines
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

        leds.initOutputLed();
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

        // leds.setDefaultCommand(leds.setLEDMode(LED_MODES.RED));

        // Reset gyro to 0° when B button is pressed
        flysky.upLTRIM.onTrue(DriveCommands.zeroDrive(this).ignoringDisable(true));
        flysky.downLTRIM.onTrue(DriveCommands.zeroDrive60(this).ignoringDisable(true));

        flysky.downLTRIM.onTrue(new InstantCommand(() -> resetSimulation()));

        // rezero superstructure
        flysky.upRTRIM.onTrue(ComplexCommands.zeroSuperstructure().ignoringDisable(true));

        // force wrist rezero
        controlBoard.shiftT.and(controlBoard.submergeT).onTrue(ComplexCommands.rezeroWrist());

        // descorees the algae after scooring a coral
        controlBoard
                .shiftT
                .negate()
                .and(controlBoard.submergeT)
                .whileTrue(ComplexCommands.descoreAlgae());

        // left trigger commands
        // gather coral camera
        // gather coral nocam
        // gather algae camera
        // gather algae nocam
        // climb down
        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(controlBoard.algaeModeT.negate()) // algae sw
                .and(flysky.topLeftSWA) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.visionCoralGather()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(controlBoard.algaeModeT.negate()) // algae sw
                .and(flysky.topLeftSWA.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.blindGatherCoral()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(controlBoard.algaeModeT) // algae sw
                .and(flysky.topLeftSWA) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.visionAlgaeGather()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.leftTriggerSWE // gather sw
                .and(flysky.rightTriggerSWG.negate()) // not scoring
                .and(controlBoard.algaeModeT) // algae sw
                .and(flysky.topLeftSWA.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.gatherAlgae(false)
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.leftTriggerSWE // gather sw
                .and(controlBoard.climbModeT) // climb sw
                .whileTrue(r.climb.setClimbVoltage(-12));

        // right trigger commands
        // score coral camera
        // score coral nocam
        // score algae camera
        // score algae nocam
        // climb up

        flysky.rightTriggerSWG // score sw
                .and(controlBoard.algaeModeT.negate()) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.visionCoralScore()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.rightTriggerSWG // score sw
                .and(controlBoard.algaeModeT.negate()) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.blindCoralScore()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.rightTriggerSWG // score sw
                .and(controlBoard.algaeModeT) // algae sw
                .and(flysky.topRightSWD) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.visionAlgaeScore()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        flysky.rightTriggerSWG // score sw
                .and(controlBoard.algaeModeT) // algae sw
                .and(flysky.topRightSWD.negate()) // cam sw
                .and(controlBoard.climbModeT.negate()) // climb sw
                .whileTrue(
                        ComplexCommands.blindAlgaeScore()
                                .alongWith(new InstantCommand(() -> state.hasStop = false)));

        // CLIMB THINGS

        flysky.rightTriggerSWG // score sw
                .and(controlBoard.climbModeT) // climb sw
                .whileTrue(r.climb.setClimbVoltage(12));

        // camera climb
        // flysky.rightTriggerSWG // score sw
        //        .and(controlBoard.climbModeT) // climb sw
        //        .whileTrue(new CmdDriveCageTraj(r));

        // get safely into climb mode
        controlBoard
                .climbModeT
                .onTrue(ComplexCommands.goToClimb().andThen(new RunCommand(() -> {})))
                .onTrue(
                        r.climb
                                .setClimbVoltage(-12)
                                .raceWith(new WaitCommand(3))
                                .finallyDo(() -> r.climb.setVolts(0)));

        // get safely out of climb position
        controlBoard.climbModeT.onFalse(ComplexCommands.leaveClimb());

        // OTHER

        // stop button
        flysky.topRightMomentSWC.whileTrue(
                ComplexCommands.stopSuperstructure()
                        .alongWith(new InstantCommand(() -> state.hasStop = true))
                        .ignoringDisable(true));

        // slot 0 for coral, 1 for algae
        controlBoard.algaeModeT.onTrue(
                new InstantCommand(() -> arm.setPIDSlot(ClosedLoopSlot.kSlot1))
                        .ignoringDisable(true));
        controlBoard.algaeModeT.onFalse(
                new InstantCommand(() -> arm.setPIDSlot(ClosedLoopSlot.kSlot0))
                        .ignoringDisable(true));

        // move to a hold position when switching modes
        controlBoard.algaeModeT.onTrue(
                new InstantCommand(() -> state.hasCoral = false)
                        .andThen(
                                ComplexCommands.goToLoc(
                                        () -> SuperstructureLocation.HOLD_ALGAE_XFER)));
        controlBoard.algaeModeT.onFalse(ComplexCommands.goToLoc(() -> SuperstructureLocation.HOLD));

        // gather and unjam
        controlBoard.gatherBtn.and(controlBoard.shiftT.negate()).onTrue(hand.setVoltageCmd(3));
        controlBoard
                .gatherBtn
                .and(controlBoard.shiftT.negate())
                .onFalse(hand.setVoltageCmd(ComplexCommands.holdPowerCoral));
        controlBoard.gatherBtn.and(controlBoard.shiftT).onTrue(hand.setVoltageCmd(-3));
        controlBoard.gatherBtn.and(controlBoard.shiftT).onFalse(hand.setVoltageCmd(0));

        // neutral switch
        neutralSwitch.onTrue(ComplexCommands.setBrakeSuperStructure(false).ignoringDisable(true));
        neutralSwitch.onFalse(ComplexCommands.setBrakeSuperStructure(true).ignoringDisable(true));
    }

    public void robotPeriodic() {
        controlBoard.periodic();
        // controlBoard.selectApproachingStation(); // TODO: delete me

        Logger.recordOutput("State/DisableBtn", !dio1.get());

        String s =
                String.format(
                        "%.1f,%.0f,%.0f",
                        r.elevator.getHeight().in(Inches),
                        r.arm.getAngle().in(Degrees),
                        r.wrist.getAngleRads().in(Degrees));
        SmartDashboard.putString("SuperPosition", s);

        Pose2d botPose = drive.getPose();
        s =
                String.format(
                        "%.0f,%.0f",
                        botPose.getMeasureX().in(Inches), botPose.getMeasureY().in(Inches));
        SmartDashboard.putString("BotLoc", s);

        SmartDashboard.putNumber("BotAngle", botPose.getRotation().getDegrees());

        /*
        double distToTag =
                r.drive
                        .getPose()
                        .getTranslation()
                        .getDistance(
                                Locations.tags.getTagPose(7).get().toPose2d().getTranslation());
        Logger.recordOutput("DistTo7", distToTag);
        */

        SmartDashboard.putNumber("ClimbPos", climb.inputs.climbAbsPosition);

        state.periodic();

        // ledValue.set(localLedVal);
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
