package frc.robot.subsystems.cvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class CVisionCals {

    public boolean disable = false;

    public double maxCageAge = 0.2; // seconds
    public int bufferSize = (int) (maxCageAge / 0.02) + 1;

    //  public double maxTagAge = 0.2;
    // the 6 inches is there to offset the front of the camera to the end of the bumper
    // need to offset along Y for the camera offset this is 12 inches
    public Pose2d camLocation =
            new Pose2d(
                    Units.inchesToMeters(12), Units.inchesToMeters(6), Rotation2d.fromDegrees(90));

    /*
    public Pose3d tagCamLocation = new Pose3d(new Translation3d(
                                                            Units.inchesToMeters(-13.25),
                                                            Units.inchesToMeters(9.5),
                                                            Units.inchesToMeters(18.5)
                                                            ),
                                                   new Rotation3d(
                                                            0,
                                                            Math.toRadians(28),
                                                            Math.PI
                                                            )
                                                    );
                                                    */
}
