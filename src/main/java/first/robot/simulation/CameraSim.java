package first.robot.simulation;

import java.util.Arrays;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;

public class CameraSim {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final NetworkTableEntry tvEntry;
  private final NetworkTableEntry txEntry;
  private final NetworkTableEntry tidEntry;
  private final NetworkTableEntry rawFiducialsEntry;

  // Camera specs (Limelight 3 FOV and max tag detection range)
  private static final double MAX_FOV_DEGREES = 31.0; // +/- 31 degrees horizontal
  private static final double MAX_RANGE_METERS = 5.0;
  private static final double MAX_RANGE_SQ = MAX_RANGE_METERS * MAX_RANGE_METERS;
  private static final int VALUES_PER_FIDUCIAL = 7;

  // Offset from AdvantageScope center-origin to WPILib Blue-Alliance origin
  private static final double FIELD_HALF_LENGTH = 8.270;
  private static final double FIELD_HALF_WIDTH = 4.035;

  // Only the AprilTags mounted on the Blue Hub and Red Hub scoring targets
  private static final Set<Integer> HUB_TAG_IDS =
      Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

  private static Pose2d tagPose(double centerX, double centerY) {
    return new Pose2d(centerX + FIELD_HALF_LENGTH, centerY + FIELD_HALF_WIDTH, Rotation2d.ZERO);
  }

  // All 32 AprilTags from Field3d_2026FRCFieldV2/config.json
  private static final Map<Integer, Pose2d> FIELD_TAGS =
      Map.ofEntries(
          Map.entry(1, tagPose(-3.6070, -3.3900)),
          Map.entry(2, tagPose(-3.6450, -0.6040)),
          Map.entry(3, tagPose(-3.0410, -0.3560)),
          Map.entry(4, tagPose(-3.0410, 0.0000)),
          Map.entry(5, tagPose(-3.6450, 0.6040)),
          Map.entry(6, tagPose(-3.6070, 3.3900)),
          Map.entry(7, tagPose(-3.6830, 3.3900)),
          Map.entry(8, tagPose(-4.0010, 0.6040)),
          Map.entry(9, tagPose(-4.2490, 0.3550)),
          Map.entry(10, tagPose(-4.2490, 0.0000)),
          Map.entry(11, tagPose(-4.0010, -0.6040)),
          Map.entry(12, tagPose(-3.6830, -3.3900)),
          Map.entry(13, tagPose(-8.2620, -3.3690)),
          Map.entry(14, tagPose(-8.2620, -2.9370)),
          Map.entry(15, tagPose(-8.2620, -0.2890)),
          Map.entry(16, tagPose(-8.2620, 0.1430)),
          Map.entry(17, tagPose(3.6070, 3.3900)),
          Map.entry(18, tagPose(3.6450, 0.6040)),
          Map.entry(19, tagPose(3.0410, 0.3550)),
          Map.entry(20, tagPose(3.0410, 0.0000)),
          Map.entry(21, tagPose(3.6450, -0.6040)),
          Map.entry(22, tagPose(3.6070, -3.3900)),
          Map.entry(23, tagPose(3.6830, -3.3900)),
          Map.entry(24, tagPose(4.0000, -0.6040)),
          Map.entry(25, tagPose(4.2490, -0.3560)),
          Map.entry(26, tagPose(4.2490, 0.0000)),
          Map.entry(27, tagPose(4.0000, 0.6040)),
          Map.entry(28, tagPose(3.6830, 3.3900)),
          Map.entry(29, tagPose(8.2620, 3.3690)),
          Map.entry(30, tagPose(8.2620, 2.9370)),
          Map.entry(31, tagPose(8.2620, 0.2890)),
          Map.entry(32, tagPose(8.2620, -0.1430)));

  private final double[] fiducialBuffer = new double[FIELD_TAGS.size() * VALUES_PER_FIDUCIAL];

  public CameraSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    this.tvEntry = limelightTable.getEntry("tv");
    this.txEntry = limelightTable.getEntry("tx");
    this.tidEntry = limelightTable.getEntry("tid");
    this.rawFiducialsEntry = limelightTable.getEntry("rawfiducials");
  }

  public void periodic() {
    Pose2d robotPose = robotPoseSupplier.get();
    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double robotHeadingRad = robotPose.getRotation().getRadians();

    int bestHubTagId = -1;
    double bestHubTx = 0.0;
    double closestHubDistance = MAX_RANGE_METERS;
    int count = 0;

    for (Map.Entry<Integer, Pose2d> entry : FIELD_TAGS.entrySet()) {
      int tagId = entry.getKey();
      Pose2d tagPose = entry.getValue();
      double deltaX = tagPose.getX() - robotX;
      double deltaY = tagPose.getY() - robotY;
      double distSq = deltaX * deltaX + deltaY * deltaY;

      if (distSq > MAX_RANGE_SQ) {
        continue;
      }

      double tx =
          MathUtil.inputModulus(
              Math.toDegrees(Math.atan2(deltaY, deltaX) - robotHeadingRad), -180.0, 180.0);

      if (Math.abs(tx) <= MAX_FOV_DEGREES) {
        double distance = Math.sqrt(distSq);
        // Standard 7-value Limelight rawfiducial packet:
        // [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity]
        double area = 1.0 / Math.max(0.01, distSq);
        fiducialBuffer[count++] = tagId;
        fiducialBuffer[count++] = tx;
        fiducialBuffer[count++] = 0.0;
        fiducialBuffer[count++] = area;
        fiducialBuffer[count++] = distance;
        fiducialBuffer[count++] = distance;
        fiducialBuffer[count++] = 0.0;

        if (HUB_TAG_IDS.contains(tagId) && distance < closestHubDistance) {
          closestHubDistance = distance;
          bestHubTagId = tagId;
          bestHubTx = tx;
        }
      }
    }

    rawFiducialsEntry.setDoubleArray(Arrays.copyOf(fiducialBuffer, count));

    if (bestHubTagId != -1) {
      tvEntry.setDouble(1.0);
      txEntry.setDouble(bestHubTx);
      tidEntry.setDouble(bestHubTagId);
    } else {
      tvEntry.setDouble(0.0);
      txEntry.setDouble(0.0);
      tidEntry.setDouble(-1);
    }
  }
}
