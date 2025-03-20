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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.Locations;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private RobotContainer r;
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    private final Debouncer isEnabledDebounce = new Debouncer(5, DebounceType.kFalling);
    private final Debouncer angleAgrees = new Debouncer(1, DebounceType.kFalling);
    LinearFilter accelFilter = LinearFilter.singlePoleIIR(1.0, 0.02);

    private Pose3d defaultPose = new Pose3d();

    public static Vision create(RobotContainer r) {
        Vision v;
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                v =
                        new Vision(
                                r.drive::addVisionMeasurement,
                                new VisionIOLimelight(camera0Name, r.drive::getRotation));
                // vision disable
                // v = new Vision(r.drive::addVisionMeasurement, new VisionIO() {});
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                v =
                        new Vision(
                                r.drive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim(
                                        camera0Name, robotToCamera0, r.drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                // (Use same number of dummy implementations as the real robot)
                v = new Vision(r.drive::addVisionMeasurement, new VisionIO() {});
                break;
        }

        v.r = r;
        return v;
    }

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + Integer.toString(i) + " is disconnected.",
                            AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public Transform2d distToPose;
    public int closestSeenTag;
    public Timer lastResetTime = new Timer();
    public double maxMemoryTime = 0.5;

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        double accelNorm = r.drive.getAccelerometer().getNorm();
        accelFilter.calculate(accelNorm);
        Logger.recordOutput("Vision/Accel", accelNorm);
        Logger.recordOutput("Vision/FiltAccel", accelFilter.lastValue());

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            if (inputs[cameraIndex].tagIds.length != 0
                    || inputs[cameraIndex].poseObservations.length != 0
                    || lastResetTime.hasElapsed(maxMemoryTime)) {
                distToPose = new Transform2d(100, 100, Rotation2d.kZero);
                closestSeenTag = 0;
                lastResetTime.restart();
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                        && observation.ambiguity()
                                                > maxAmbiguity) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                        > maxZError // Must have realistic Z coordinate

                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth()

                                // only use mega1 when angle is bad and robot not moving for a while
                                || observation.type() == PoseObservationType.MEGATAG_1
                        /*&& (isMoving()
                        || isEnabled()
                        || angleAgrees(
                                observation
                                        .pose()
                                        .getRotation()
                                        .toRotation2d()))*/ ;

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    Logger.recordOutput(
                            "Vision/RejectTagDist",
                            observation
                                    .pose()
                                    .toPose2d()
                                    .minus(r.drive.getPose())
                                    .getTranslation()
                                    .getNorm());
                    Logger.recordOutput(
                            "Vision/RejectTagAngle",
                            observation
                                    .pose()
                                    .getRotation()
                                    .toRotation2d()
                                    .minus(r.drive.getRotation())
                                    .getDegrees());
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor =
                        Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

                // find closest tag in shot to robot

                Pose2d observationPose = observation.pose().toPose2d();
                for (int i = 0; i < inputs[cameraIndex].tagIds.length; i++) {
                    int id = inputs[cameraIndex].tagIds[i];
                    var tagPose = aprilTagLayout.getTagPose(id).orElse(defaultPose).toPose2d();
                    Transform2d dist = tagPose.minus(observationPose);
                    if (dist.getTranslation().getNorm() < distToPose.getTranslation().getNorm()) {
                        distToPose = dist;
                        closestSeenTag = id;
                    }
                }

                Logger.recordOutput("Vision/ClosestTagDist", distToPose.getTranslation().getNorm());
                Logger.recordOutput(
                        "Vision/ClosestTagAngle", distToPose.getRotation().getDegrees());
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses",
                allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    public boolean selectedTagOnTarget() {
        int targetId = Locations.getTagId(r.controlBoard.selectedReefPos);
        if (targetId == closestSeenTag) {
            if (distToPose.getTranslation().getNorm() < Units.inchesToMeters(20)) {
                return true;
            }
        }
        return false;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }

    double movingThreshold = 0.25; // m/s^2

    private boolean isMoving() {
        boolean isMovingVal =
                Math.abs(r.drive.getAccelerometer().getNorm() - accelFilter.lastValue())
                        > movingThreshold;
        Logger.recordOutput("Vision/IsMoving", isMovingVal);
        return isMovingVal;
    }

    private boolean isEnabled() {
        boolean isEnabledVal = (isEnabledDebounce.calculate(DriverStation.isEnabled()));
        Logger.recordOutput("Vision/IsEnabled", isEnabledVal);
        return isEnabledVal;
    }

    private boolean angleAgrees(Rotation2d tagAngle) {
        Rotation2d deltaAngle = r.drive.getRotation().minus(tagAngle);
        boolean angleAgreesVal = (angleAgrees.calculate(Math.abs(deltaAngle.getDegrees()) < 4));
        Logger.recordOutput("Vision/AngleAgrees", angleAgreesVal);
        return angleAgreesVal;
    }
}
