/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.simulation;

import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;

public final class Field2026Physics {
  // Field dimensions matching CameraSim (2 * 8.270m x 2 * 4.035m)
  public static final double FIELD_LENGTH = 16.540;
  public static final double FIELD_WIDTH = 8.070;

  // Robot collision footprint & wheel sample offsets (meters)
  public static final double ROBOT_HALF_LENGTH = 0.42;
  public static final double ROBOT_HALF_WIDTH = 0.38;
  public static final double ROBOT_COLLISION_RADIUS = 0.42;
  private static final double ROBOT_COLLISION_RADIUS_SQ =
      ROBOT_COLLISION_RADIUS * ROBOT_COLLISION_RADIUS;

  // Hub & Neutral-Zone Blocker geometry derived from CameraSim / field layout
  public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.625, 4.035);
  public static final Translation2d RED_HUB_CENTER = new Translation2d(11.915, 4.035);
  public static final double HUB_HALF_SIZE = 0.604;
  public static final double HUB_FUNNEL_HEIGHT = 1.83;
  public static final double HUB_FUNNEL_RADIUS = 0.56;
  public static final double BLOCKER_HALF_DEPTH = 0.22;
  public static final double BLOCKER_HALF_WIDTH = 0.62;
  public static final double BLOCKER_HEIGHT = 2.40;
  public static final double SIDE_BLOCKER_WIDTH = 0.42;
  public static final double SIDE_BLOCKER_HEIGHT = 1.83;
  public static final double TRENCH_WALL_HEIGHT = 0.40;

  // Bump geometry (sloped ramps flanking each Hub between Hub and Trench)
  private static final double BUMP_HALF_LENGTH_X = 0.605;
  private static final double BUMP_PEAK_HEIGHT = 0.165; // ~6.5 inches
  private static final double BUMP_SLOPE_MAGNITUDE = BUMP_PEAK_HEIGHT / BUMP_HALF_LENGTH_X;
  private static final double BUMP_INNER_Y_OFFSET = 0.62; // Just outside Hub
  private static final double BUMP_OUTER_Y_OFFSET = 2.68; // Up to Trench divider

  /** Axis-aligned 3D bounding box for solid field obstacles. */
  public record BoxCollider(
      double minX, double maxX, double minY, double maxY, double height, boolean isHub) {
    public static BoxCollider fromCenter(double cx, double cy, double halfX, double halfY) {
      return new BoxCollider(
          cx - halfX, cx + halfX, cy - halfY, cy + halfY, HUB_FUNNEL_HEIGHT, false);
    }

    public static BoxCollider fromCenter(
        double cx, double cy, double halfX, double halfY, double height, boolean isHub) {
      return new BoxCollider(cx - halfX, cx + halfX, cy - halfY, cy + halfY, height, isHub);
    }

    public boolean contains(double x, double y, double margin) {
      return x >= (minX - margin)
          && x <= (maxX + margin)
          && y >= (minY - margin)
          && y <= (maxY + margin);
    }
  }

  /** Vertical prism with a 2D triangular footprint in the X-Y plane (small side blockers). */
  public record TrianglePrismCollider(
      double ax, double ay, double bx, double by, double cx, double cy, double height) {

    /** Returns the closest point on the filled 2D triangle ABC to (px, py). */
    public Translation2d closestPoint2d(double px, double py) {
      if (contains2d(px, py)) {
        return new Translation2d(px, py);
      }
      return closestPerimeterPoint2d(px, py);
    }

    /** Returns the closest point on the boundary edges of triangle ABC to (px, py). */
    public Translation2d closestPerimeterPoint2d(double px, double py) {
      Translation2d pAB = closestPointOnSegment(px, py, ax, ay, bx, by);
      Translation2d pBC = closestPointOnSegment(px, py, bx, by, cx, cy);
      Translation2d pCA = closestPointOnSegment(px, py, cx, cy, ax, ay);

      double dxAB = px - pAB.getX();
      double dyAB = py - pAB.getY();
      double dSqAB = dxAB * dxAB + dyAB * dyAB;

      double dxBC = px - pBC.getX();
      double dyBC = py - pBC.getY();
      double dSqBC = dxBC * dxBC + dyBC * dyBC;

      double dxCA = px - pCA.getX();
      double dyCA = py - pCA.getY();
      double dSqCA = dxCA * dxCA + dyCA * dyCA;

      if (dSqAB <= dSqBC && dSqAB <= dSqCA) {
        return pAB;
      } else if (dSqBC <= dSqCA) {
        return pBC;
      } else {
        return pCA;
      }
    }

    /** Returns true if (px, py) lies inside or on the boundary of triangle ABC. */
    public boolean contains2d(double px, double py) {
      double d1 = cross2d(px, py, ax, ay, bx, by);
      double d2 = cross2d(px, py, bx, by, cx, cy);
      double d3 = cross2d(px, py, cx, cy, ax, ay);

      boolean hasNeg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
      boolean hasPos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);
      return !(hasNeg && hasPos);
    }

    public Translation2d centroid2d() {
      return new Translation2d((ax + bx + cx) / 3.0, (ay + by + cy) / 3.0);
    }

    private static double cross2d(
        double px, double py, double x1, double y1, double x2, double y2) {
      return (px - x2) * (y1 - y2) - (x1 - x2) * (py - y2);
    }

    private static Translation2d closestPointOnSegment(
        double px, double py, double x1, double y1, double x2, double y2) {
      double vx = x2 - x1;
      double vy = y2 - y1;
      double lenSq = vx * vx + vy * vy;
      if (lenSq < 1e-12) {
        return new Translation2d(x1, y1);
      }
      double t = Math.clamp(((px - x1) * vx + (py - y1) * vy) / lenSq, 0.0, 1.0);
      return new Translation2d(x1 + t * vx, y1 + t * vy);
    }
  }

  // Solid field obstacles: Blue/Red Hubs, Neutral-Zone Hub Blockers, and the 4 Trench divider walls
  public static final List<BoxCollider> SOLID_OBSTACLES =
      List.of(
          // Blue Hub & Red Hub bases
          BoxCollider.fromCenter(
              BLUE_HUB_CENTER.getX(),
              BLUE_HUB_CENTER.getY(),
              HUB_HALF_SIZE,
              HUB_HALF_SIZE,
              HUB_FUNNEL_HEIGHT,
              true),
          BoxCollider.fromCenter(
              RED_HUB_CENTER.getX(),
              RED_HUB_CENTER.getY(),
              HUB_HALF_SIZE,
              HUB_HALF_SIZE,
              HUB_FUNNEL_HEIGHT,
              true),
          // Blue & Red Neutral-Zone Hub Blockers (shields attached to the Neutral-Zone face of each
          // Hub)
          BoxCollider.fromCenter(
              BLUE_HUB_CENTER.getX() + HUB_HALF_SIZE + BLOCKER_HALF_DEPTH,
              BLUE_HUB_CENTER.getY(),
              BLOCKER_HALF_DEPTH,
              BLOCKER_HALF_WIDTH,
              BLOCKER_HEIGHT,
              false),
          BoxCollider.fromCenter(
              RED_HUB_CENTER.getX() - HUB_HALF_SIZE - BLOCKER_HALF_DEPTH,
              RED_HUB_CENTER.getY(),
              BLOCKER_HALF_DEPTH,
              BLOCKER_HALF_WIDTH,
              BLOCKER_HEIGHT,
              false),
          // Trench divider walls separating Bumps from outer Trench lanes
          BoxCollider.fromCenter(4.625, 1.28, 0.60, 0.08, TRENCH_WALL_HEIGHT, false),
          BoxCollider.fromCenter(4.625, 6.79, 0.60, 0.08, TRENCH_WALL_HEIGHT, false),
          BoxCollider.fromCenter(11.915, 1.28, 0.60, 0.08, TRENCH_WALL_HEIGHT, false),
          BoxCollider.fromCenter(11.915, 6.79, 0.60, 0.08, TRENCH_WALL_HEIGHT, false));

  // Small triangular side blocker wings flanking the left (+Y) and right (-Y) edges of each Hub
  // blocker
  public static final List<TrianglePrismCollider> TRIANGLE_SIDE_BLOCKERS =
      createTriangleSideBlockers();

  private static List<TrianglePrismCollider> createTriangleSideBlockers() {
    double blueBaseX = BLUE_HUB_CENTER.getX() + HUB_HALF_SIZE;
    double blueTipX = blueBaseX + 2.0 * BLOCKER_HALF_DEPTH;
    double blueY = BLUE_HUB_CENTER.getY();

    double redBaseX = RED_HUB_CENTER.getX() - HUB_HALF_SIZE;
    double redTipX = redBaseX - 2.0 * BLOCKER_HALF_DEPTH;
    double redY = RED_HUB_CENTER.getY();

    return List.of(
        // Blue Left (+Y) & Right (-Y) triangle side blockers
        new TrianglePrismCollider(
            blueBaseX,
            blueY + BLOCKER_HALF_WIDTH,
            blueBaseX,
            blueY + BLOCKER_HALF_WIDTH + SIDE_BLOCKER_WIDTH,
            blueTipX,
            blueY + BLOCKER_HALF_WIDTH,
            SIDE_BLOCKER_HEIGHT),
        new TrianglePrismCollider(
            blueBaseX,
            blueY - BLOCKER_HALF_WIDTH,
            blueBaseX,
            blueY - BLOCKER_HALF_WIDTH - SIDE_BLOCKER_WIDTH,
            blueTipX,
            blueY - BLOCKER_HALF_WIDTH,
            SIDE_BLOCKER_HEIGHT),
        // Red Left (+Y) & Right (-Y) triangle side blockers
        new TrianglePrismCollider(
            redBaseX,
            redY + BLOCKER_HALF_WIDTH,
            redBaseX,
            redY + BLOCKER_HALF_WIDTH + SIDE_BLOCKER_WIDTH,
            redTipX,
            redY + BLOCKER_HALF_WIDTH,
            SIDE_BLOCKER_HEIGHT),
        new TrianglePrismCollider(
            redBaseX,
            redY - BLOCKER_HALF_WIDTH,
            redBaseX,
            redY - BLOCKER_HALF_WIDTH - SIDE_BLOCKER_WIDTH,
            redTipX,
            redY - BLOCKER_HALF_WIDTH,
            SIDE_BLOCKER_HEIGHT));
  }

  private Field2026Physics() {}

  /**
   * Resolves 2D circle-vs-perimeter and circle-vs-obstacle collisions for the robot across 2 solver
   * passes so adjacent box/triangle seams never trap the chassis.
   */
  public static Pose2d resolveRobotCollisions(Pose2d rawPose) {
    double x = rawPose.getX();
    double y = rawPose.getY();

    for (int pass = 0; pass < 2; pass++) {
      x = Math.clamp(x, ROBOT_COLLISION_RADIUS, FIELD_LENGTH - ROBOT_COLLISION_RADIUS);
      y = Math.clamp(y, ROBOT_COLLISION_RADIUS, FIELD_WIDTH - ROBOT_COLLISION_RADIUS);

      for (BoxCollider box : SOLID_OBSTACLES) {
        double closestX = Math.clamp(x, box.minX(), box.maxX());
        double closestY = Math.clamp(y, box.minY(), box.maxY());
        double dx = x - closestX;
        double dy = y - closestY;
        double distSq = dx * dx + dy * dy;

        if (distSq < ROBOT_COLLISION_RADIUS_SQ) {
          double dist = Math.sqrt(distSq);
          if (dist > 1e-6) {
            double push = ROBOT_COLLISION_RADIUS - dist;
            x += (dx / dist) * push;
            y += (dy / dist) * push;
          } else {
            // Center inside box fallback: push out along nearest edge
            double penLeft = Math.abs(x - box.minX());
            double penRight = Math.abs(box.maxX() - x);
            double penBottom = Math.abs(y - box.minY());
            double penTop = Math.abs(box.maxY() - y);
            double minPen = Math.min(Math.min(penLeft, penRight), Math.min(penBottom, penTop));
            if (minPen == penLeft) x = box.minX() - ROBOT_COLLISION_RADIUS;
            else if (minPen == penRight) x = box.maxX() + ROBOT_COLLISION_RADIUS;
            else if (minPen == penBottom) y = box.minY() - ROBOT_COLLISION_RADIUS;
            else y = box.maxY() + ROBOT_COLLISION_RADIUS;
          }
        }
      }

      for (TrianglePrismCollider tri : TRIANGLE_SIDE_BLOCKERS) {
        if (tri.contains2d(x, y)) {
          Translation2d edgePt = tri.closestPerimeterPoint2d(x, y);
          double dx = edgePt.getX() - x;
          double dy = edgePt.getY() - y;
          double distSq = dx * dx + dy * dy;
          if (distSq > 1e-12) {
            double dist = Math.sqrt(distSq);
            double scale = (dist + ROBOT_COLLISION_RADIUS) / dist;
            x += dx * scale;
            y += dy * scale;
          } else {
            Translation2d centroid = tri.centroid2d();
            double outX = x - centroid.getX();
            double outY = y - centroid.getY();
            double outDist = Math.hypot(outX, outY);
            if (outDist > 1e-6) {
              x += (outX / outDist) * ROBOT_COLLISION_RADIUS;
              y += (outY / outDist) * ROBOT_COLLISION_RADIUS;
            }
          }
        } else {
          Translation2d closest = tri.closestPerimeterPoint2d(x, y);
          double dx = x - closest.getX();
          double dy = y - closest.getY();
          double distSq = dx * dx + dy * dy;
          if (distSq < ROBOT_COLLISION_RADIUS_SQ && distSq > 1e-12) {
            double dist = Math.sqrt(distSq);
            double push = ROBOT_COLLISION_RADIUS - dist;
            x += (dx / dist) * push;
            y += (dy / dist) * push;
          }
        }
      }
    }

    return new Pose2d(x, y, rawPose.getRotation());
  }

  /** Returns the 3D floor height (Z in meters) at any (x, y) coordinate on the field. */
  public static double getTerrainHeight(double x, double y) {
    double distFromCenterY = Math.abs(y - (FIELD_WIDTH / 2.0));
    if (distFromCenterY < BUMP_INNER_Y_OFFSET || distFromCenterY > BUMP_OUTER_Y_OFFSET) {
      return 0.0;
    }

    double blueDx = Math.abs(x - BLUE_HUB_CENTER.getX());
    if (blueDx < BUMP_HALF_LENGTH_X) {
      return BUMP_PEAK_HEIGHT * (1.0 - (blueDx / BUMP_HALF_LENGTH_X));
    }

    double redDx = Math.abs(x - RED_HUB_CENTER.getX());
    if (redDx < BUMP_HALF_LENGTH_X) {
      return BUMP_PEAK_HEIGHT * (1.0 - (redDx / BUMP_HALF_LENGTH_X));
    }

    return 0.0;
  }

  /** Returns the terrain X-slope (dz/dx) at (x, y), or 0.0 on flat floor. */
  public static double getBumpSlopeX(double x, double y) {
    double distFromCenterY = Math.abs(y - (FIELD_WIDTH / 2.0));
    if (distFromCenterY < BUMP_INNER_Y_OFFSET || distFromCenterY > BUMP_OUTER_Y_OFFSET) {
      return 0.0;
    }

    double blueDx = x - BLUE_HUB_CENTER.getX();
    if (Math.abs(blueDx) < BUMP_HALF_LENGTH_X) {
      return blueDx >= 0.0 ? -BUMP_SLOPE_MAGNITUDE : BUMP_SLOPE_MAGNITUDE;
    }

    double redDx = x - RED_HUB_CENTER.getX();
    if (Math.abs(redDx) < BUMP_HALF_LENGTH_X) {
      return redDx >= 0.0 ? -BUMP_SLOPE_MAGNITUDE : BUMP_SLOPE_MAGNITUDE;
    }

    return 0.0;
  }

  /**
   * Computes a full 3D robot pose (Z height, roll, and pitch) by sampling terrain heights under all
   * four corners of the chassis.
   */
  public static Pose3d computeRobotPose3d(Pose2d pose2d) {
    Rotation2d yaw = pose2d.getRotation();
    Translation2d center = pose2d.getTranslation();

    double zFL = sampleCornerHeight(center, yaw, ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH);
    double zFR = sampleCornerHeight(center, yaw, ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH);
    double zRL = sampleCornerHeight(center, yaw, -ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH);
    double zRR = sampleCornerHeight(center, yaw, -ROBOT_HALF_LENGTH, -ROBOT_HALF_WIDTH);

    double avgZ = (zFL + zFR + zRL + zRR) / 4.0;
    double frontZ = (zFL + zFR) / 2.0;
    double rearZ = (zRL + zRR) / 2.0;
    double leftZ = (zFL + zRL) / 2.0;
    double rightZ = (zFR + zRR) / 2.0;

    // Positive pitch in WPILib tilts the nose (+X) downward, so uphill (frontZ > rearZ) is negative
    double pitch = -Math.atan2(frontZ - rearZ, 2.0 * ROBOT_HALF_LENGTH);
    // Positive roll in WPILib lifts the left side (+Y), so left-side-up (leftZ > rightZ) is
    // positive
    double roll = Math.atan2(leftZ - rightZ, 2.0 * ROBOT_HALF_WIDTH);

    return new Pose3d(
        pose2d.getX(), pose2d.getY(), avgZ, new Rotation3d(roll, pitch, yaw.getRadians()));
  }

  private static double sampleCornerHeight(
      Translation2d center, Rotation2d yaw, double localX, double localY) {
    Translation2d worldCorner = center.plus(new Translation2d(localX, localY).rotateBy(yaw));
    return getTerrainHeight(worldCorner.getX(), worldCorner.getY());
  }
}
