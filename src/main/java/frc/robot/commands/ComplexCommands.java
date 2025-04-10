package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ComplexCommands {

    public static double holdPowerCoral = 0.4;
    static double releasePowerCoral23 = -1.75;
    static double releasePowerCoral4 = -4 - 1;
    public static double releasePowerCoral1 = -2.5;
    static double releaseTimeCoral1 = 0.3;
    static double releaseTimeCoral23 = 0.25;
    static double releaseTimeCoral4 = 0.04 + 0.06;
    public static double releasePowerCoralClimb = -1.25;

    public static double intakePowerCoral = 3.0; // 4
    public static double intakeCurrentCoral = 15;
    static double intakeCoralTime = 0.8;

    static double intakeAlgaeTime = 0.2;
    static double intakeCurrentAlgae = 50;
    static double intakeAlgaePower = 10;
    static double holdAlgaePower = 2;
    static double releasePowerAlgae = -12;
    static double releaseTimeAlgae = 0.5;
    static double netAngle = 60;
    static double superSuckAlgae = 6;
    static double algaeHighLim = 90;
    static double algaeLowLim = 40;
    static double descoreAlgaePower = -4;
    static double stripTime = 0.75; // make smaller

    static double gatherPosition = 0;

    static double pulseGatherOn = 0.25;
    static double pulseGatherOff = 0.15;
    static double pulseGatherOffPwr = -0.2; // 0.05

    public static RobotContainer r;

    // ALGAE COMMANDS

    public static Command stripAlgae() {
        SequentialCommandGroup sq =
                new SequentialCommandGroup(
                        r.wrist
                                .goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3)
                                .alongWith(r.arm.goTo(() -> SuperstructureLocation.VERT_ALGAE)),
                        r.elevator.goTo(r.controlBoard::getAlgaeReefDSHeight),
                        r.arm.goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3));

        Command c =
                // hand goes up
                sq.alongWith(DriveCommands.driveToPoint(r, r.controlBoard::getAlgaePathPose, false))
                        .andThen(r.hand.setVoltageCmd(descoreAlgaePower))
                        .andThen(
                                r.elevator
                                        .goTo(r.controlBoard::getAlgaeReefDSHeightLower)
                                        .alongWith(
                                                DriveCommands.driveToPoint(
                                                        r,
                                                        Locations.supercycleOffset(
                                                                r.controlBoard::getAlgaePathPose),
                                                        false)))
                        .finallyDo(() -> r.hand.setVoltage(0));

        c.setName("StripAlgae");
        return c;
    }

    public static Command oldStripAlgae() {
        SequentialCommandGroup sq =
                new SequentialCommandGroup(
                        r.wrist
                                .goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3)
                                .alongWith(r.arm.goTo(() -> SuperstructureLocation.VERT_ALGAE)),
                        r.elevator.goTo(r.controlBoard::getAlgaeReefDSHeight),
                        r.arm.goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3));

        Command c =
                // hand goes up
                sq.alongWith(DriveCommands.driveToPoint(r, r.controlBoard::getAlgaePathPose, false))
                        .andThen(r.hand.setVoltageCmd(descoreAlgaePower))
                        .andThen(
                                new WaitCommand(stripTime)
                                        .andThen(
                                                r.elevator.goTo(
                                                        r.controlBoard::getAlgaeReefDSHeightLower))
                                        .alongWith(DriveCommands.joystickDriveFlysky(r)))
                        .finallyDo(() -> r.hand.setVoltage(0));

        c.setName("StripAlgae");
        return c;
    }

    public static Command supercycleGather() {
        final boolean[] hasGathered = new boolean[1];

        SequentialCommandGroup moveArmlol =
                new SequentialCommandGroup(
                        r.arm.goTo(() -> SuperstructureLocation.HOLD)
                                .alongWith(r.elevator.goTo(r.controlBoard::getAlgaeReefHeight)),
                        r.wrist
                                .goTo(() -> SuperstructureLocation.ALGAE_LEVEL_2_3)
                                .alongWith(
                                        r.arm.goTo(() -> SuperstructureLocation.ALGAE_LEVEL_2_3)));

        SequentialCommandGroup mainSQ =
                new SequentialCommandGroup(
                        new InstantCommand(() -> hasGathered[0] = false),
                        r.hand.setVoltageCmd(intakeAlgaePower),
                        DriveCommands.driveToPoint(
                                        r,
                                        Locations.supercycleOffset(
                                                r.controlBoard::getAlgaePathPose),
                                        false)
                                .alongWith(moveArmlol),
                        DriveCommands.driveToPoint(r, r.controlBoard::getAlgaePathPose, false),
                        new InstantCommand(
                                () -> {
                                    hasGathered[0] = true;
                                    r.state.setAlgae();
                                }),
                        new WaitCommand(0.1),
                        new PathFollowingCommand(r, () -> r.pathCache.closestWaypoint(), true));

        Command c = mainSQ.finallyDo(() -> r.controlBoard.tempScoreAlgae = hasGathered[0]);

        c.setName("SupercycleGather");
        return c;
    }

    public static Command releaseAlgae() {
        Command c =
                r.hand.setVoltageCmd(releasePowerAlgae)
                        .andThen(new WaitCommand(releaseTimeAlgae))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseAlgae");
        return c;
    }

    public static Command holdAlgae() {
        Command c = r.hand.setVoltageCmd(holdAlgaePower);
        c.setName("HoldAlgae");
        return c;
    }

    public static Command scoreAlgaeNet() {
        // lots of horizontal vel, lauches from PRENET

        SequentialCommandGroup launch = new SequentialCommandGroup();
        launch.addCommands(r.hand.setCurrentLimCmd(algaeHighLim));
        launch.addCommands(r.hand.setVoltageCmd(superSuckAlgae));
        launch.addCommands(new WaitCommand(0.1));
        launch.addCommands(
                forceGoToLoc(() -> SuperstructureLocation.NET)
                        .raceWith(
                                new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) < netAngle)
                                        .andThen(r.hand.setVoltageCmd(releasePowerAlgae))
                                        .andThen(new WaitCommand(0.5))));
        launch.setName("launch");

        Command c =
                goToLocAlgae(() -> SuperstructureLocation.PRENET)
                        .andThen(
                                new WaitUntilCommand(
                                        r.flysky.leftTriggerSWE.and(r.state.pathCompleteT)))
                        .andThen(launch)
                        .andThen(r.hand.stop())
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(
                                () -> {
                                    r.hand.setVoltage(0);
                                    r.hand.setCurrentLim(algaeLowLim);
                                    r.state.hasAlgae = false;
                                });

        c.setName("ScoreAlgaeNet");
        return c;
    }

    public static Command scoreAlgaeNet2() {
        // less horizontal vel, launches from HOLD_ALGAE_IN

        SequentialCommandGroup launch = new SequentialCommandGroup();
        launch.addCommands(goToAlgaeHold());
        launch.addCommands(
                new WaitUntilCommand(
                        r.flysky.leftTriggerSWE.and(r.state.pathCompleteT).or(r.shootForNet)));
        launch.addCommands(new InstantCommand(() -> r.state.algaeNetStage2 = true));

        launch.setName("ScoreAlgaeNet2");
        return launch;
    }

    public static Command netLaunch() {
        double minEleVel = 10; // in/sec

        SequentialCommandGroup scg = new SequentialCommandGroup();
        scg.addCommands(new WaitCommand(0));
        scg.addCommands(r.hand.setCurrentLimCmd(algaeHighLim));
        scg.addCommands(r.hand.setVoltageCmd(superSuckAlgae));
        scg.addCommands(new WaitCommand(0.1));

        // launch elevator arm
        scg.addCommands(
                new InstantCommand(
                        () -> {
                            // r.arm.setVoltage(-2.5);
                            r.elevator.setVoltage(4.5);
                        }));

        // wait to release ball
        scg.addCommands(
                new WaitUntilCommand(() -> r.elevator.getHeight().in(Inches) > 26)
                        .raceWith(new WaitCommand(0.25)));
        scg.addCommands(r.hand.setVoltageCmd(releasePowerAlgae));

        // wait until ball is out
        scg.addCommands(
                new WaitCommand(0.08)
                        .raceWith(
                                new WaitUntilCommand(
                                        () ->
                                                r.elevator.getHeight().in(Inches) > 41
                                                        || r.arm.getAngle().in(Degrees) < -50)));

        // slow down
        scg.addCommands(
                new InstantCommand(
                        () -> {
                            // r.arm.setVoltage(1);
                            r.elevator.setVoltage(-1);
                        }));
        scg.addCommands(
                new WaitCommand(0.2)
                        .raceWith(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.04),
                                                new WaitUntilCommand(
                                                        () -> r.elevator.getVelocity() < minEleVel),
                                                new InstantCommand(
                                                        () -> r.elevator.setVoltage(0))) /*,
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(
                                                        () -> {
                                                            double delta =
                                                                    r.arm.getAngle().in(Degrees)
                                                                            - armPosition[0];
                                                            armPosition[0] =
                                                                    r.arm.getAngle().in(Degrees);
                                                            return delta > 0;
                                                        }),
                                                new InstantCommand(() -> r.arm.setVoltage(0)))*/)));

        // stop
        scg.addCommands(r.hand.setCurrentLimCmd(algaeHighLim));
        scg.addCommands(
                new InstantCommand(
                        () -> {
                            SuperstructureLocation loc = SuperstructureLocation.VERT_ALGAE;
                            r.arm.setAngle(loc.armAngle);
                            r.elevator.setHeight(loc.eleHeight);
                            r.wrist.setAngle(loc);
                            r.hand.setVoltage(0);
                            r.state.algaeNetStage2 = false;
                            r.state.hasAlgae = false;
                        },
                        r.elevator,
                        r.arm,
                        r.wrist,
                        r.hand));

        // super hack, watch out
        Command pathWrapper =
                new FunctionalCommand(
                        () -> Logger.recordOutput("State/PathFirst", false),
                        () -> {
                            if (r.activePath != null) {
                                // System.out.println("Path Not Null");
                                r.activePath.execute();
                            } else {
                                // System.out.println("Path was null");
                            }
                        },
                        (interrupted) -> {
                            if (r.activePath != null) {
                                if (!interrupted) {
                                    Logger.recordOutput("State/PathFirst", true);
                                }
                                r.activePath.end(interrupted);
                                r.activePath = null;
                            }
                        },
                        () -> {
                            if (r.activePath != null) {
                                return r.activePath.isFinished();
                            } else {
                                return false;
                            }
                        },
                        r.drive);

        Command c =
                scg.deadlineFor(pathWrapper)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        c.setName("NetLaunch");
        return c;
    }

    public static Command scoreAlgaeProc() {
        Command c =
                goToLocAlgae(() -> SuperstructureLocation.SCORE_PROCESSOR)
                        .andThen(
                                new WaitUntilCommand(
                                        r.flysky.leftTriggerSWE.and(r.state.pathCompleteT)))
                        .andThen(releaseAlgae())
                        .andThen(new RunCommand(() -> {}));
        c.setName("ScoreAlgaeProc");
        return c;
    }

    public static Command gatherAlgae(boolean cmaMode) {
        Command c =
                goToLocAlgae(
                                () ->
                                        cmaMode
                                                ? r.controlBoard.getAlgaeReefHeight()
                                                : r.controlBoard.getAlgaeLevel())
                        .andThen(r.hand.setVoltageCmd(intakeAlgaePower))
                        // .until(() -> r.hand.getCurrent() > intakeCurrentAlgae)
                        .andThen(new InstantCommand(() -> r.state.setAlgae()))
                        .andThen(new RunCommand(() -> {})) // never end
                        .finallyDo(() -> r.hand.setVoltage(holdAlgaePower));
        c.setName("GatherAlgae");
        return c;
    }

    public static Command visionAlgaeGather() {
        Command c =
                DriveCommands.driveToAuto(r, r.controlBoard::getAlgaePathPose, false)
                        .raceWith(
                                new WaitUntilCommand(r.inSlowDrivePhase).andThen(gatherAlgae(true)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                                new PathFollowingCommand(
                                        r, () -> r.pathCache.closestWaypoint(), true))
                        .andThen(goToAlgaeHold());
        c.setName("VisionAlgaeGather");
        return c;
    }

    public static Command blindAlgaeScore() {
        Command c =
                new ConditionalCommand(
                        scoreAlgaeNet2(), scoreAlgaeProc(), r.controlBoard::getAlgaeScoreLoc);
        c.setName("BlindAlgaeScore");
        return c;
    }

    public static Command visionAlgaeScore() {
        Command c =
                new ConditionalCommand(
                        visionScoreAlgaeNet2(),
                        visionScoreAlgaeProc(),
                        // () -> r.controlBoard.selectedLevel == 4);
                        r.controlBoard::getAlgaeScoreLoc);

        c.setName("VisionAlgaeScore");
        return c;
    }

    public static Command visionScoreAlgaeNet() {
        Command c =
                DriveCommands.driveTo(r, () -> Locations.getNetPose(r.drive.getGlobalPose()), true)
                        .alongWith(new WaitCommand(0.25).andThen(scoreAlgaeNet2()));
        c.setName("visionScoreAlgaeNet");
        return c;
    }

    public static Command visionScoreAlgaeNet2() {
        Command c = new BargePathFinder(r).alongWith(scoreAlgaeNet2());
        c.setName("visionScoreAlgaeNet");
        return c;
    }

    public static Command visionScoreAlgaeProc() {
        Command c =
                DriveCommands.driveTo(r, Locations::getProcLoc, false)
                        .alongWith(new WaitCommand(0.25).andThen(scoreAlgaeProc()));
        // wait a sec to leave the reef before dropping elevator
        c.setName("VisionScoreAlgaeProc");
        return c;
    }

    public static Command descoreAlgae() {
        Command c =
                goToLoc(r.controlBoard::getAlgaeDescoreLevel)
                        .andThen(r.hand.setVoltageCmd(4))
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(() -> r.hand.setVoltage(0));
        c.setName("descoreAlgae");
        return c;
    }

    // CORAL COMMANDS

    public static Command gatherCoral() {
        Command c =
                r.hand.setVoltageCmd(intakePowerCoral)
                        .andThen(new WaitCommand(0.5))
                        .andThen(
                                new WaitUntilCommand(
                                        () -> r.hand.getCurrent() > intakeCurrentCoral))
                        .andThen(new WaitCommand(intakeCoralTime))
                        .andThen(new InstantCommand(() -> r.state.setCoral()))
                        .finallyDo(() -> r.hand.setVoltage(holdPowerCoral));
        c.setName("GatherCoral");
        return c;
    }

    public static Command gatherCoral2() {
        Command c =
                pulseGather()
                        .finallyDo(
                                () -> {
                                    r.state.setCoral();
                                });

        c.setName("GatherCoral2");
        return c;
    }

    public static Command pulseGather() {
        Command c =
                new WaitCommand(pulseGatherOn)
                        .deadlineFor(r.hand.setVoltageCmd(intakePowerCoral))
                        .andThen(
                                new WaitCommand(pulseGatherOff)
                                        .deadlineFor(r.hand.setVoltageCmd(pulseGatherOffPwr)))
                        .repeatedly()
                        .finallyDo(() -> r.hand.setVoltage(holdPowerCoral));

        c.setName("PulseGather");
        return c;
    }

    // applies power to get rid of coral\
    public static Command releaseCoral() {
        Command c =
                new InstantCommand(
                                () -> {
                                    switch (r.controlBoard.selectedLevel) {
                                        case 1:
                                            r.hand.setVoltage(releasePowerCoral1);
                                            break;
                                        case 2:
                                        case 3:
                                        default:
                                            r.hand.setVoltage(releasePowerCoral23);
                                            break;
                                        case 4:
                                            r.hand.setVoltage(releasePowerCoral4);
                                            break;
                                    }
                                })
                        .andThen(
                                new DynamicWaitCommand(
                                        () -> {
                                            switch (r.controlBoard.selectedLevel) {
                                                case 1:
                                                    return releaseTimeCoral1;
                                                case 2:
                                                case 3:
                                                    return releaseTimeCoral23;
                                                case 4:
                                                default:
                                                    return releaseTimeCoral4;
                                            }
                                        }))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseCoral");
        return c;
    }

    // applies power to get rid of coral in auton
    public static Command releaseCoralAuton(int level) {

        Command c = // this doesnt work in teleop, but who cares, would need to be a supplier
                (level == 4
                                ? r.hand.setVoltageCmd(releasePowerCoral4)
                                : r.hand.setVoltageCmd(releasePowerCoral23))
                        .andThen(
                                new DynamicWaitCommand(
                                        () -> {
                                            switch (r.controlBoard.selectedLevel) {
                                                case 1:
                                                    return releaseTimeCoral1;
                                                case 2:
                                                case 3:
                                                    return releaseTimeCoral23;
                                                case 4:
                                                default:
                                                    return releaseTimeCoral4;
                                            }
                                        }))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseCoralAuton");
        return c;
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        Command c = r.hand.setVoltageCmd(holdPowerCoral);
        c.setName("HoldCoral");
        return c;
    }

    public static Command noDriveGather() {
        Command fixTempAlgae =
                new ConditionalCommand(
                                r.hand.setVoltageCmd(0),
                                new InstantCommand(),
                                r.controlBoard.tempGatherAlgaeT.or(r.controlBoard.tempScoreAlgaeT))
                        .andThen(
                                new InstantCommand(
                                        () -> {
                                            r.controlBoard.tempGatherAlgae = false;
                                            r.controlBoard.tempScoreAlgae = false;
                                        }));

        Command c = goToGather().alongWith(fixTempAlgae).andThen(gatherCoral2());
        // if we were in algae mode, make sure we drop the algae before getting to
        // coral intake position

        c.setName("NoDriveGather");
        return c;
    }

    public static Command noDriveScore() {
        Command c =
                goToLoc(() -> r.controlBoard.getCoralLevel())
                        .alongWith(holdCoral())
                        // gather trigger
                        .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                        .andThen(releaseCoral());
        c.setName("NoDriveScore");
        return c;
    }

    public static Command visionCoralGather() {
        // decides which coral station to use
        // drive there
        // gather
        Command c =
                DriveCommands.driveTo(r, r.controlBoard::selectCoralStation, true)
                        .alongWith(noDriveGather());
        c.setName("VisionCoralGather");
        return c;
    }

    public static Command blindCoralScore() {
        Command snap = snapToAngle(r.controlBoard::getAlignAngle).alongWith(noDriveScore());
        Command justDrive = DriveCommands.joystickDriveFlysky(r).alongWith(noDriveScore());
        Command c =
                new ConditionalCommand(justDrive, snap, () -> r.controlBoard.selectedLevel == 1);
        c.setName("BlindCoralScore");
        return c;
    }

    public static Command blindGatherCoral() {
        Command c = snapToAngle(r.controlBoard::selectGatherAngle).alongWith(noDriveGather());
        c.setName("BlindGatherCoral");
        return c;
    }

    public static Command visionCoralScore() {
        Command c =
                DriveCommands.driveTo(r, r.controlBoard::getPathPose, false)
                        .raceWith(
                                new WaitUntilCommand(r.inSlowDrivePhase)
                                        .andThen(
                                                goToLoc(r.controlBoard::getCoralLevel)
                                                        .alongWith(holdCoral())
                                                        .andThen(
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                r.flysky
                                                                                        .leftTriggerSWE
                                                                                        .and(
                                                                                                r.state
                                                                                                        .pathCompleteT)
                                                                                        .or(
                                                                                                r.state
                                                                                                        .onTargetT)
                                                                                        .getAsBoolean())))
                                        .andThen(releaseCoral()))
                        .andThen(
                                new ConditionalCommand(
                                        supercycleGather(),
                                        new InstantCommand(),
                                        r.controlBoard.tempGatherAlgaeT))
                        .andThen(
                                new ConditionalCommand(
                                        stripAlgae(),
                                        new InstantCommand(),
                                        () ->
                                                !r.flysky.botRightSWHLo.getAsBoolean()
                                                        && r.controlBoard.selectedLevel != 1));
        c.setName("VisionCoralScore");
        return c;
    }

    // IMPORTANT/COMMONLY USED

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        final boolean[] midGather = new boolean[1];

        Command notInGather =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command inGatherOld =
                r.elevator
                        .goTo(() -> SuperstructureLocation.PRE_INTAKE)
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.POST_INTAKE))
                        .andThen(r.wrist.goTo(() -> SuperstructureLocation.POST_INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.POST_INTAKE2))
                        .andThen(r.wrist.goTo(() -> SuperstructureLocation.POST_INTAKE2))
                        .andThen(
                                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                                        .alongWith(r.wrist.goTo(() -> SuperstructureLocation.HOLD)))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command inGather =
                r.elevator
                        .goTo(() -> SuperstructureLocation.PRE_INTAKE)
                        // .andThen(new PrintCommand("starting lift"))
                        .andThen(
                                r.wrist
                                        .setVoltage(0.5)
                                        .alongWith(r.arm.setVoltageCmd(2))
                                        .alongWith(new InstantCommand(() -> midGather[0] = true)))
                        .andThen(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) > -15))
                        // .andThen(new PrintCommand("lift complete"))
                        .andThen(
                                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                                        .alongWith(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                                        .alongWith(new InstantCommand(() -> midGather[0] = false)))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command c =
                new ConditionalCommand(
                        inGather,
                        notInGather,
                        () -> atLocation(SuperstructureLocation.INTAKE, true));

        // make sure we stop driving the arm/wrist with voltage
        c =
                c.finallyDo(
                        () -> {
                            if (midGather[0]) {
                                r.wrist.goTo(() -> SuperstructureLocation.HOLD).execute();
                                r.arm.goTo(() -> SuperstructureLocation.HOLD).execute();
                                midGather[0] = false;
                            }
                        });
        c.setName("GoToLoc");
        return c;
    }

    // no fancy protection logic, force to position
    public static Command forceGoToLoc(Supplier<SuperstructureLocation> p) {
        Command c = new ParallelCommandGroup(r.elevator.goTo(p), r.arm.goTo(p), r.wrist.goTo(p));
        c.setName("ForceGoToLoc");
        return c;
    }

    public static Command goToLocAlgae(Supplier<SuperstructureLocation> p) {
        // no longer matters as we are out of coral gather when the switch is flipped
        Command toAlgaeFromCoralOld =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                        .andThen(r.elevator.goTo(() -> SuperstructureLocation.HOLD_ALGAE_OUT))
                        .andThen(
                                r.elevator
                                        .goTo(p)
                                        .alongWith(r.arm.goTo(p).alongWith(r.wrist.goTo(p))));

        // safely get to algae gather position from center position
        Command toAlgaeFromCenter =
                r.arm.goTo(() -> SuperstructureLocation.HOLD_ALGAE_OUT)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD_ALGAE_OUT))
                        .andThen(
                                r.elevator
                                        .goTo(p)
                                        .alongWith(r.arm.goTo(p))
                                        .alongWith(r.wrist.goTo(p)));

        Command toAlgaeFromAlgae = forceGoToLoc(p);

        Command c =
                new ConditionalCommand(
                        toAlgaeFromCenter,
                        toAlgaeFromAlgae,
                        () -> r.arm.getAngle().in(Degrees) < 50);
        c.setName("goToLocAlgae");
        return c;
    }

    // safely get to algae hold position from algae gather positions
    public static Command goToAlgaeHold() {
        final SuperstructureLocation hold = SuperstructureLocation.HOLD_ALGAE_IN;

        SequentialCommandGroup scg = new SequentialCommandGroup();

        // go up first
        scg.addCommands(r.elevator.goTo(() -> hold));
        scg.addCommands(r.arm.goTo(() -> hold).alongWith(r.wrist.goTo(() -> hold)));
        scg.addCommands(r.elevator.goTo(() -> hold));

        scg.setName("goToHoldAlgae");

        // dont move if already there
        return new ConditionalCommand(
                new InstantCommand(),
                scg,
                () -> r.elevator.atTarget(() -> hold, true) && r.arm.atTarget(() -> hold, true));
    }

    // lines up with target field element
    public static Command snapToAngle(Supplier<Rotation2d> angle) {
        Command c =
                DriveCommands.joystickDriveAtAngle(
                        r,
                        () -> -r.flysky.getLeftY(),
                        () -> -r.flysky.getLeftX(),
                        () -> -r.flysky.getRightX(),
                        angle);
        c.setName("SnapToAngle");
        return c;
    }

    // go to hold, algae hold, or gather depending on what game pieces we have
    public static Command homeLogic() {
        Command c =
                new ConditionalCommand(
                                new RunCommand(() -> {}),
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new RunCommand(() -> {}),
                                                goToLoc(() -> SuperstructureLocation.HOLD)
                                                        .andThen(new RunCommand(() -> {})),
                                                () ->
                                                        atLocation(
                                                                SuperstructureLocation.INTAKE,
                                                                true)),
                                        new ConditionalCommand(
                                                (new RunCommand(() -> {})),
                                                goToGather()
                                                        .andThen(
                                                                new RunCommand(
                                                                        () -> {})), // prevent
                                                // command from
                                                // ending
                                                r.controlBoard.algaeModeT), // coral/algae sw
                                        r.state.hasCoralT),
                                r.state.hasStopT.or(r.controlBoard.climbModeT))
                        .andThen(new WaitUntilCommand(() -> false));
        c.setName("HomeLogic");
        return c;
    }

    // MINI COMMANDS

    public static Command goToGather() {
        Command toGather =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .alongWith(r.wrist.goToReally(() -> SuperstructureLocation.HOLD))
                        .andThen(
                                r.wrist
                                        .goToReally(() -> SuperstructureLocation.HOLD_GATHER)
                                        .raceWith(new WaitCommand(0.75)))
                        .alongWith(r.elevator.goTo(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.wrist.goToReally(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.elevator.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.wrist.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(new InstantCommand(() -> r.state.setCoral()));

        Command c =
                new ConditionalCommand(
                        rawGoTo(() -> SuperstructureLocation.INTAKE), // go directly to intake
                        toGather, // go to gather position via sequence
                        () -> atLocation(SuperstructureLocation.INTAKE, true));
        c.setName("GoToGather");
        return c;
    }

    // just goes to, nothing fancy, no protections
    public static Command rawGoTo(Supplier<SuperstructureLocation> loc) {
        return r.elevator.goTo(loc).alongWith(r.arm.goTo(loc)).alongWith(r.wrist.goTo(loc));
    }

    public static Command goToClimb() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(goToLoc(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.hand.setVoltageCmd(releasePowerCoralClimb));
        c.addCommands(r.wrist.setVoltage(-1.5));
        c.addCommands(new WaitCommand(0.75));
        c.addCommands(r.wrist.setVoltage(-0.6));
        c.addCommands(r.arm.setVoltageCmd(-0.75));
        c.addCommands(r.hand.setVoltageCmd(0));
        c.addCommands(new WaitCommand(1));
        c.addCommands(r.wrist.setVoltage(-0.1));
        c.addCommands(r.arm.setVoltageCmd(-0.1));

        c.setName("GoToClimb");
        return c;
    }

    public static Command leaveClimb() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(r.wrist.setVoltage(0));
        c.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.arm.setVoltageCmd(2));
        c.addCommands(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) > -10));
        c.addCommands(r.arm.setVoltageCmd(0));
        c.addCommands(r.wrist.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.arm.goTo(() -> SuperstructureLocation.HOLD));

        c.setName("LeaveClimb");
        return c;
    }

    public static Command stopSuperstructure() {
        Command c = r.elevator.stop().alongWith(r.arm.stop()).alongWith(r.wrist.stop());
        // .alongWith(r.hand.stop());
        c.setName("StopSuperstructure");
        return c;
    }

    public static boolean atLocation(SuperstructureLocation loc, boolean extraTolerance) {
        return r.arm.atTarget(() -> loc, extraTolerance)
                && r.elevator.atTarget(() -> loc, extraTolerance)
                && r.wrist.atTarget(() -> loc, extraTolerance);
    }

    public static Command zeroSuperstructure() {
        Command c =
                new InstantCommand(
                        () -> {
                            r.elevator.zero();
                            r.arm.zero();
                            r.wrist.zero();
                        });
        c.setName("ZeroSuperstructure");
        return c;
    }

    public static Command freeBird() {
        SequentialCommandGroup liftArm = new SequentialCommandGroup();
        liftArm.addCommands(r.wrist.setVoltage(-1));
        liftArm.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        liftArm.addCommands(new WaitCommand(1.5));
        liftArm.addCommands(r.arm.setVoltageCmd(2));
        liftArm.addCommands(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) > -10));

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(r.wrist.setVoltage(0.01));
        c.addCommands(
                new ConditionalCommand(
                        liftArm, new InstantCommand(), () -> r.arm.getAngle().in(Degrees) < -10));
        c.addCommands(
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .alongWith(r.elevator.goTo(() -> SuperstructureLocation.HOLD))
                        .alongWith(r.wrist.goTo(() -> SuperstructureLocation.HOLD)));
        c.addCommands(r.hand.stop());

        c.setName("FreeBird");
        return c;
    }

    public static Command rezeroWrist() {
        SequentialCommandGroup liftArm = new SequentialCommandGroup();
        liftArm.addCommands(r.wrist.setVoltage(-1));
        liftArm.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        liftArm.addCommands(new WaitCommand(1.5));
        liftArm.addCommands(r.arm.setVoltageCmd(2));
        liftArm.addCommands(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) > -10));

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(r.wrist.setVoltage(0.01));
        c.addCommands(
                new ConditionalCommand(
                        liftArm, new InstantCommand(), () -> r.arm.getAngle().in(Degrees) < -10));
        c.addCommands(r.arm.goTo(() -> SuperstructureLocation.ZERO_WRIST));
        c.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.hand.stop());
        c.addCommands(r.wrist.setVoltage(-1));
        c.addCommands(new WaitCommand(2));
        c.addCommands(r.wrist.stop());
        c.addCommands(new InstantCommand(() -> r.wrist.rezeroAbsEnc()));
        c.addCommands(new WaitCommand(0.2));

        // c.addRequirements(r.elevator, r.arm, r.wrist, r.hand);
        c.setName("RezeroWrist");
        Command cmd = c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        cmd = cmd.raceWith(new WaitCommand(10));

        return cmd;
    }

    public static Command setBrakeSuperStructure(boolean on) {
        return new InstantCommand(
                () -> {
                    r.arm.setBrake(on);
                    r.elevator.setBrake(on);
                    r.wrist.setBrake(on);
                    r.climb.setBrake(on);
                },
                r.arm,
                r.elevator,
                r.wrist,
                r.climb);
    }
}
