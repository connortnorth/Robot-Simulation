/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.opmode;

import first.robot.Robot;
import first.robot.simulation.Field2026Physics;
import first.robot.simulation.FuelSim;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous
public class RedHubAuto extends PeriodicOpMode {
  private enum AutoState {
    COLLECT_FUEL,
    NAVIGATE_TO_RED_HUB,
    ALIGN_AND_SHOOT,
    DONE
  }

  private static final class PathResult {
    final List<Translation2d> waypoints;
    final double totalCost;

    PathResult(List<Translation2d> waypoints, double totalCost) {
      this.waypoints = waypoints;
      this.totalCost = totalCost;
    }
  }

  private static final class IgnoredBall {
    final Translation2d position;
    double expiryTimeSeconds;

    IgnoredBall(Translation2d position, double expiryTimeSeconds) {
      this.position = position;
      this.expiryTimeSeconds = expiryTimeSeconds;
    }
  }

  private final Robot robot;
  private final NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTable autoTable = NetworkTableInstance.getDefault().getTable("Auto");

  // AprilTags mounted on the Red Hub (18..21 on West/Neutral side, 24..27 on East/Red side)
  private static final Set<Integer> RED_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

  // Hopper fill target (fills 3x3 base + 3 second-layer balls right up to the overflow rim)
  private static final int TARGET_FULL_HOPPER = 12;

  // Shooting & launcher parameters matching MyTeleop
  private static final double IDEAL_HUB_CENTER_DISTANCE_METERS = 1.80;
  private static final double LAUNCH_THROTTLE = 0.90;
  private static final double FEED_THROTTLE = 0.75;
  private static final double LAUNCH_READY_RPM = 4900.0;
  private static final double BALL_SHOT_MARGIN = 0.18;

  // Navigation & A* grid parameters
  private static final double GRID_RES = 0.25;
  private static final int GRID_COLS =
      (int) Math.ceil(Field2026Physics.FIELD_LENGTH / GRID_RES) + 1;
  private static final int GRID_ROWS = (int) Math.ceil(Field2026Physics.FIELD_WIDTH / GRID_RES) + 1;
  private static final double CLEARANCE_RADIUS = Field2026Physics.ROBOT_COLLISION_RADIUS + 0.08;
  private static final double BUMP_TRAVERSAL_PENALTY = 2.4;

  // Drive & alignment PID controllers
  private final PIDController turnPID = new PIDController(0.035, 0.0, 0.001);
  private final PIDController alignDrivePID = new PIDController(1.0, 0.0, 0.08);
  private final PIDController pathDrivePID = new PIDController(1.35, 0.0, 0.05);

  private AutoState state = AutoState.COLLECT_FUEL;
  private Translation2d currentTargetBall = null;
  private List<Translation2d> activePath = List.of();
  private int waypointIndex = 0;
  private double elapsedSeconds = 0.0;
  private double ballChaseTimer = 0.0;
  private double stallTimer = 0.0;
  private Translation2d stallAnchorPos = null;
  private double postShotTimer = 0.0;
  private int scoringCyclesCompleted = 0;
  private final List<IgnoredBall> ignoredBalls = new ArrayList<>();

  public RedHubAuto(Robot robot) {
    this.robot = robot;
    turnPID.setTolerance(1.5);
    alignDrivePID.setTolerance(0.05);
  }

  @Override
  public void periodic() {
    elapsedSeconds += 0.02;
    ignoredBalls.removeIf(b -> elapsedSeconds >= b.expiryTimeSeconds);

    Pose2d robotPose = robot.getPose();
    autoTable.getEntry("State").setString(state.name());
    autoTable.getEntry("CyclesCompleted").setDouble(scoringCyclesCompleted);

    switch (state) {
      case COLLECT_FUEL -> runCollectFuel(robotPose);
      case NAVIGATE_TO_RED_HUB -> runNavigateToRedHub(robotPose);
      case ALIGN_AND_SHOOT -> runAlignAndShoot(robotPose);
      case DONE -> runDone();
    }
  }

  private void runCollectFuel(Pose2d robotPose) {
    int storedCount = FuelSim.getStoredFuelCount();
    List<Translation2d> fieldBalls = FuelSim.getCollectibleFieldFuelPositions();

    // Head to the Red Hub once hopper is full, or if no reachable balls remain on the field
    if (storedCount >= TARGET_FULL_HOPPER || fieldBalls.isEmpty()) {
      if (storedCount == 0) {
        state = AutoState.DONE;
        runDone();
        return;
      }
      transitionToNavigateToRedHub(robotPose);
      return;
    }

    // Run intake rollers while sweeping nearest balls
    robot.intakeLauncher.setThrottle(0.85);
    robot.feeder.setThrottle(-1.0);

    Translation2d robotPos = robotPose.getTranslation();
    Translation2d bestBall = selectNearestReachableBall(robotPose, fieldBalls);
    if (bestBall == null) {
      if (storedCount > 0) {
        transitionToNavigateToRedHub(robotPose);
      } else {
        state = AutoState.DONE;
        runDone();
      }
      return;
    }

    boolean targetChanged =
        currentTargetBall == null || currentTargetBall.getDistance(bestBall) > 0.22;
    if (targetChanged) {
      currentTargetBall = bestBall;
      ballChaseTimer = 0.0;
      resetStallTracker(robotPos);
      replanPathTo(robotPos, currentTargetBall);
    } else {
      currentTargetBall = bestBall;
      ballChaseTimer += 0.02;
      if (ballChaseTimer > 3.5) {
        // Skip balls wedged against walls/corners so the robot never gets stuck
        ignoredBalls.add(new IgnoredBall(currentTargetBall, elapsedSeconds + 5.0));
        currentTargetBall = null;
        activePath = List.of();
        ballChaseTimer = 0.0;
        return;
      }
      if (activePath.isEmpty() || updateStallDetected(robotPos, 0.55)) {
        replanPathTo(robotPos, currentTargetBall);
      }
    }

    while (waypointIndex < activePath.size() - 1
        && robotPos.getDistance(activePath.get(waypointIndex)) < 0.34) {
      waypointIndex++;
    }

    Translation2d steerTarget =
        activePath.isEmpty() ? currentTargetBall : activePath.get(waypointIndex);
    double dx = steerTarget.getX() - robotPos.getX();
    double dy = steerTarget.getY() - robotPos.getY();
    double distToWaypoint = Math.hypot(dx, dy);
    double distToBall = robotPos.getDistance(currentTargetBall);

    // Point the front intake (+X) toward the next waypoint / ball
    Rotation2d desiredHeading = new Rotation2d(Math.atan2(dy, dx));
    double angleErrorDeg =
        MathUtil.inputModulus(
            desiredHeading.minus(robotPose.getRotation()).getDegrees(), -180.0, 180.0);
    double rotSpeed = Math.clamp(-turnPID.calculate(angleErrorDeg, 0.0), -0.85, 0.85);

    double vxRobot;
    double vyRobot;
    if (distToBall < 1.10
        && Math.abs(angleErrorDeg) < 28.0
        && isSegmentClear(robotPos, currentTargetBall)) {
      // Drive straight through the ball so the front intake captures it cleanly
      vxRobot = 0.65;
      vyRobot = 0.0;
    } else if (distToWaypoint > 1e-4) {
      double speedScale = Math.clamp(pathDrivePID.calculate(0.0, distToWaypoint), 0.38, 0.85);
      double turnAttenuation = Math.max(0.45, Math.cos(Math.toRadians(angleErrorDeg)));
      Translation2d robotDir =
          new Translation2d(dx / distToWaypoint, dy / distToWaypoint)
              .rotateBy(robotPose.getRotation().unaryMinus());
      vxRobot = robotDir.getX() * speedScale * turnAttenuation;
      vyRobot = robotDir.getY() * speedScale * turnAttenuation;
    } else {
      vxRobot = 0.0;
      vyRobot = 0.0;
    }

    robot.drivetrain.drive(vxRobot, vyRobot, rotSpeed, false);
  }

  private void transitionToNavigateToRedHub(Pose2d robotPose) {
    currentTargetBall = null;
    activePath = List.of();
    waypointIndex = 0;
    ballChaseTimer = 0.0;
    resetStallTracker(robotPose.getTranslation());
    state = AutoState.NAVIGATE_TO_RED_HUB;
    runNavigateToRedHub(robotPose);
  }

  private void runNavigateToRedHub(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d hubCenter = Field2026Physics.RED_HUB_CENTER;
    boolean hasClearShot = isShotLineClear(robotPos, hubCenter);

    // Keep intake running briefly until any ball mid-ramp finishes entering the hopper
    if (FuelSim.getIntakingFuelCount() > 0) {
      robot.intakeLauncher.setThrottle(0.80);
      robot.feeder.setThrottle(-1.0);
    } else if (hasClearShot) {
      // Pre-spin launcher only once past the blocker on the Red Alliance side
      robot.intakeLauncher.setThrottle(LAUNCH_THROTTLE);
      robot.feeder.setThrottle(0.0);
    } else {
      robot.intakeLauncher.setThrottle(0.0);
      robot.feeder.setThrottle(0.0);
    }

    // Plan route once on entry, and only replan if progress stalls
    if (activePath.isEmpty() || updateStallDetected(robotPos, 0.60)) {
      PathResult bestRoute = findBestRouteToRedHub(robotPos);
      activePath = trimReachedLeadingWaypoints(robotPos, bestRoute.waypoints);
      waypointIndex = 0;
      publishPath(activePath);
    }

    while (waypointIndex < activePath.size() - 1
        && robotPos.getDistance(activePath.get(waypointIndex)) < 0.35) {
      waypointIndex++;
    }

    Translation2d finalGoal =
        activePath.isEmpty()
            ? hubCenter.plus(new Translation2d(IDEAL_HUB_CENTER_DISTANCE_METERS, 0.0))
            : activePath.get(activePath.size() - 1);
    double distToGoal = robotPos.getDistance(finalGoal);
    double distToHub = robotPos.getDistance(hubCenter);

    // Transition to precision shooting only on the Red-Alliance side with a clear shot line
    if (hasClearShot
        && Field2026Physics.getTerrainHeight(robotPos.getX(), robotPos.getY()) == 0.0
        && (distToGoal < 0.35 || (distToHub <= 2.10 && distToHub >= 1.50))) {
      activePath = List.of();
      turnPID.reset();
      alignDrivePID.reset();
      state = AutoState.ALIGN_AND_SHOOT;
      runAlignAndShoot(robotPose);
      return;
    }

    Translation2d target = activePath.isEmpty() ? finalGoal : activePath.get(waypointIndex);
    double dx = target.getX() - robotPos.getX();
    double dy = target.getY() - robotPos.getY();
    double distToTarget = Math.hypot(dx, dy);

    // Face the Red Hub once past the blocker within 2.8m; otherwise face direction of travel
    Rotation2d desiredHeading =
        (distToHub < 2.8 && hasClearShot)
            ? new Rotation2d(
                Math.atan2(hubCenter.getY() - robotPos.getY(), hubCenter.getX() - robotPos.getX()))
            : new Rotation2d(Math.atan2(dy, dx));

    double angleErrorDeg =
        MathUtil.inputModulus(
            desiredHeading.minus(robotPose.getRotation()).getDegrees(), -180.0, 180.0);
    double rotSpeed = Math.clamp(-turnPID.calculate(angleErrorDeg, 0.0), -0.85, 0.85);

    double vxRobot = 0.0;
    double vyRobot = 0.0;
    if (distToTarget > 1e-4) {
      double speed = Math.clamp(pathDrivePID.calculate(0.0, distToGoal), 0.38, 0.90);
      Translation2d robotDir =
          new Translation2d(dx / distToTarget, dy / distToTarget)
              .rotateBy(robotPose.getRotation().unaryMinus());
      vxRobot = robotDir.getX() * speed;
      vyRobot = robotDir.getY() * speed;
    }

    robot.drivetrain.drive(vxRobot, vyRobot, rotSpeed, false);
  }

  private void runAlignAndShoot(Pose2d robotPose) {
    // Prefer live Red Hub AprilTag detection, falling back to known Red Hub coordinates while
    // turning
    Translation2d hubCenter = getVisibleRedHubCenter();
    if (hubCenter == null) {
      hubCenter = Field2026Physics.RED_HUB_CENTER;
    }

    Translation2d robotPos = robotPose.getTranslation();
    boolean clearShot = isShotLineClear(robotPos, hubCenter);
    if (!clearShot) {
      // Reposition around the blocker if pushed into an occluded spot
      transitionToNavigateToRedHub(robotPose);
      return;
    }

    double dx = hubCenter.getX() - robotPos.getX();
    double dy = hubCenter.getY() - robotPos.getY();
    double distanceToCenter = Math.hypot(dx, dy);

    Rotation2d angleToCenter = new Rotation2d(Math.atan2(dy, dx));
    double angleErrorDeg =
        MathUtil.inputModulus(
            angleToCenter.minus(robotPose.getRotation()).getDegrees(), -180.0, 180.0);
    double distanceError = distanceToCenter - IDEAL_HUB_CENTER_DISTANCE_METERS;

    double rotationSpeed =
        Math.abs(angleErrorDeg) > 1.0
            ? Math.clamp(-turnPID.calculate(angleErrorDeg, 0.0), -0.80, 0.80)
            : 0.0;

    double xSpeed = 0.0;
    double ySpeed = 0.0;
    if (distanceToCenter > 1e-4 && Math.abs(distanceError) > 0.04) {
      double radialSpeed =
          Math.clamp(
              -alignDrivePID.calculate(distanceToCenter, IDEAL_HUB_CENTER_DISTANCE_METERS),
              -0.75,
              0.75);
      Translation2d robotRelativeDir =
          new Translation2d(dx / distanceToCenter, dy / distanceToCenter)
              .rotateBy(robotPose.getRotation().unaryMinus());
      xSpeed = robotRelativeDir.getX() * radialSpeed;
      ySpeed = robotRelativeDir.getY() * radialSpeed;
    }

    robot.drivetrain.drive(xSpeed, ySpeed, rotationSpeed, false);

    // Spin up launcher flywheel fully before feeding
    robot.intakeLauncher.setThrottle(LAUNCH_THROTTLE);
    Double launcherRpm = robot.intakeLauncher.getEncoder().getVelocity().get();
    boolean flywheelReady = launcherRpm != null && launcherRpm >= LAUNCH_READY_RPM;
    boolean aligned = Math.abs(angleErrorDeg) <= 2.5 && Math.abs(distanceError) <= 0.12;

    int remainingFuel = FuelSim.getStoredFuelCount();
    if (remainingFuel > 0) {
      postShotTimer = 0.0;
      if (flywheelReady && aligned) {
        robot.feeder.setThrottle(FEED_THROTTLE);
      } else {
        robot.feeder.setThrottle(0.0);
      }
    } else {
      // Hold briefly after final ball exits so it clears the flywheel cleanly, then go collect more
      robot.feeder.setThrottle(0.0);
      postShotTimer += 0.02;
      if (postShotTimer >= 0.25) {
        scoringCyclesCompleted++;
        postShotTimer = 0.0;
        currentTargetBall = null;
        activePath = List.of();
        waypointIndex = 0;
        turnPID.reset();
        alignDrivePID.reset();
        resetStallTracker(robotPos);

        if (!FuelSim.getCollectibleFieldFuelPositions().isEmpty()) {
          state = AutoState.COLLECT_FUEL;
        } else {
          state = AutoState.DONE;
        }
      }
    }
  }

  private void runDone() {
    robot.drivetrain.stop();
    robot.intakeLauncher.setThrottle(0.0);
    robot.feeder.setThrottle(0.0);
  }

  private void replanPathTo(Translation2d start, Translation2d goal) {
    if (isSegmentClear(start, goal)) {
      activePath = List.of(goal);
      waypointIndex = 0;
    } else {
      PathResult result = findSmartPath(start, goal);
      activePath = trimReachedLeadingWaypoints(start, result.waypoints);
      waypointIndex = 0;
    }
    publishPath(activePath);
  }

  private List<Translation2d> trimReachedLeadingWaypoints(
      Translation2d robotPos, List<Translation2d> waypoints) {
    int startIdx = 0;
    while (startIdx < waypoints.size() - 1
        && robotPos.getDistance(waypoints.get(startIdx)) < 0.32) {
      startIdx++;
    }
    return waypoints.subList(startIdx, waypoints.size());
  }

  private void resetStallTracker(Translation2d pos) {
    stallAnchorPos = pos;
    stallTimer = 0.0;
  }

  private boolean updateStallDetected(Translation2d currentPos, double timeoutSeconds) {
    if (stallAnchorPos == null || currentPos.getDistance(stallAnchorPos) >= 0.12) {
      stallAnchorPos = currentPos;
      stallTimer = 0.0;
      return false;
    }
    stallTimer += 0.02;
    if (stallTimer >= timeoutSeconds) {
      stallAnchorPos = currentPos;
      stallTimer = 0.0;
      return true;
    }
    return false;
  }

  private boolean isBallIgnored(Translation2d ball) {
    for (IgnoredBall ignored : ignoredBalls) {
      if (ignored.position.getDistance(ball) < 0.30) {
        return true;
      }
    }
    return false;
  }

  private Translation2d selectNearestReachableBall(
      Pose2d robotPose, List<Translation2d> fieldBalls) {
    Translation2d robotPos = robotPose.getTranslation();
    Translation2d bestBall = null;
    double bestCost = Double.MAX_VALUE;

    for (Translation2d ball : fieldBalls) {
      if (isBallIgnored(ball) || !isPositionClear(ball.getX(), ball.getY())) {
        continue;
      }
      double dist = robotPos.getDistance(ball);
      double headingToBall =
          Math.atan2(ball.getY() - robotPos.getY(), ball.getX() - robotPos.getX());
      double anglePenalty =
          0.22
              * Math.abs(
                  MathUtil.angleModulus(headingToBall - robotPose.getRotation().getRadians()));
      double obstaclePenalty = isSegmentClear(robotPos, ball) ? 0.0 : 3.2;
      double hysteresisBonus =
          (currentTargetBall != null && currentTargetBall.getDistance(ball) < 0.22) ? -0.45 : 0.0;

      double cost = dist + anglePenalty + obstaclePenalty + hysteresisBonus;
      if (cost < bestCost) {
        bestCost = cost;
        bestBall = ball;
      }
    }
    return bestBall;
  }

  private PathResult findBestRouteToRedHub(Translation2d start) {
    Translation2d hub = Field2026Physics.RED_HUB_CENTER;
    // Flat, unobstructed Red-Alliance shooting angles facing the Red Hub's Red-side AprilTags
    double[] candidateAnglesDeg = {0.0, 20.0, -20.0, 35.0, -35.0};

    PathResult bestResult = null;
    for (double deg : candidateAnglesDeg) {
      double rad = Math.toRadians(deg);
      Translation2d candidate =
          new Translation2d(
              hub.getX() + IDEAL_HUB_CENTER_DISTANCE_METERS * Math.cos(rad),
              hub.getY() + IDEAL_HUB_CENTER_DISTANCE_METERS * Math.sin(rad));
      if (!isPositionClear(candidate.getX(), candidate.getY())
          || !isShotLineClear(candidate, hub)
          || Field2026Physics.getTerrainHeight(candidate.getX(), candidate.getY()) > 0.0) {
        continue;
      }
      PathResult result = findSmartPath(start, candidate);
      if (bestResult == null || result.totalCost < bestResult.totalCost) {
        bestResult = result;
      }
    }

    if (bestResult != null) {
      return bestResult;
    }
    Translation2d fallback = hub.plus(new Translation2d(IDEAL_HUB_CENTER_DISTANCE_METERS, 0.0));
    return findSmartPath(start, fallback);
  }

  /**
   * Returns true if a ball launched from shooterPos toward hubCenter has a clean Red-Alliance shot
   * path without crossing the Neutral-Zone blocker, side deflectors, or Trench walls.
   */
  private boolean isShotLineClear(Translation2d shooterPos, Translation2d hubCenter) {
    // Must shoot from the Red-Alliance side of the Hub center, never from behind the Neutral
    // blocker
    if (shooterPos.getX() < hubCenter.getX() + 0.15) {
      return false;
    }
    double dist = shooterPos.getDistance(hubCenter);
    int samples = Math.max(4, (int) Math.ceil(dist / 0.08));
    for (int i = 0; i <= samples; i++) {
      double t = (double) i / samples;
      double x = shooterPos.getX() + t * (hubCenter.getX() - shooterPos.getX());
      double y = shooterPos.getY() + t * (hubCenter.getY() - shooterPos.getY());
      for (Field2026Physics.BoxCollider box : Field2026Physics.SOLID_OBSTACLES) {
        if (box.isHub()) {
          continue;
        }
        if (box.contains(x, y, BALL_SHOT_MARGIN)) {
          return false;
        }
      }
      for (Field2026Physics.TrianglePrismCollider tri : Field2026Physics.TRIANGLE_SIDE_BLOCKERS) {
        Translation2d closest = tri.closestPoint2d(x, y);
        if (Math.hypot(x - closest.getX(), y - closest.getY()) < BALL_SHOT_MARGIN) {
          return false;
        }
      }
    }
    return true;
  }

  /** Computes an obstacle-free, bump-aware A* path smoothed via line-of-sight pruning. */
  private PathResult findSmartPath(Translation2d start, Translation2d goal) {
    if (isSegmentClear(start, goal) && !segmentCrossesBump(start, goal)) {
      return new PathResult(List.of(goal), start.getDistance(goal));
    }

    int startCol = clampCol((int) Math.round(start.getX() / GRID_RES));
    int startRow = clampRow((int) Math.round(start.getY() / GRID_RES));
    int goalCol = clampCol((int) Math.round(goal.getX() / GRID_RES));
    int goalRow = clampRow((int) Math.round(goal.getY() / GRID_RES));

    int[] startNode = nearestWalkableCell(startCol, startRow);
    int[] goalNode = nearestWalkableCell(goalCol, goalRow);
    startCol = startNode[0];
    startRow = startNode[1];
    goalCol = goalNode[0];
    goalRow = goalNode[1];

    int totalCells = GRID_COLS * GRID_ROWS;
    double[] gScore = new double[totalCells];
    double[] fScore = new double[totalCells];
    int[] cameFrom = new int[totalCells];
    boolean[] closed = new boolean[totalCells];

    for (int i = 0; i < totalCells; i++) {
      gScore[i] = Double.POSITIVE_INFINITY;
      fScore[i] = Double.POSITIVE_INFINITY;
      cameFrom[i] = -1;
    }

    int startIdx = startRow * GRID_COLS + startCol;
    int goalIdx = goalRow * GRID_COLS + goalCol;
    gScore[startIdx] = 0.0;
    fScore[startIdx] = Math.hypot((goalCol - startCol) * GRID_RES, (goalRow - startRow) * GRID_RES);

    PriorityQueue<Integer> openSet =
        new PriorityQueue<>(Comparator.comparingDouble(i -> fScore[i]));
    openSet.add(startIdx);

    while (!openSet.isEmpty()) {
      int current = openSet.poll();
      if (current == goalIdx) {
        List<Translation2d> rawPath = reconstructPath(cameFrom, current, goal);
        List<Translation2d> smoothed = smoothPath(start, rawPath);
        return new PathResult(smoothed, gScore[goalIdx]);
      }
      if (closed[current]) {
        continue;
      }
      closed[current] = true;

      int c = current % GRID_COLS;
      int r = current / GRID_COLS;

      for (int dc = -1; dc <= 1; dc++) {
        for (int dr = -1; dr <= 1; dr++) {
          if (dc == 0 && dr == 0) {
            continue;
          }
          int nc = c + dc;
          int nr = r + dr;
          if (nc < 0 || nc >= GRID_COLS || nr < 0 || nr >= GRID_ROWS) {
            continue;
          }
          // Prevent diagonal corner cutting across blocked cells
          if (dc != 0 && dr != 0 && (!isCellWalkable(c + dc, r) || !isCellWalkable(c, r + dr))) {
            continue;
          }
          if (!isCellWalkable(nc, nr)) {
            continue;
          }

          int neighbor = nr * GRID_COLS + nc;
          if (closed[neighbor]) {
            continue;
          }

          double nx = nc * GRID_RES;
          double ny = nr * GRID_RES;
          double stepDist = Math.hypot(dc, dr) * GRID_RES;
          double terrainMultiplier =
              Field2026Physics.getTerrainHeight(nx, ny) > 0.0 ? BUMP_TRAVERSAL_PENALTY : 1.0;

          double tentativeG = gScore[current] + stepDist * terrainMultiplier;
          if (tentativeG < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeG;
            double h = Math.hypot((goalCol - nc) * GRID_RES, (goalRow - nr) * GRID_RES);
            fScore[neighbor] = tentativeG + h;
            openSet.add(neighbor);
          }
        }
      }
    }

    return new PathResult(List.of(goal), start.getDistance(goal) + 10.0);
  }

  private List<Translation2d> reconstructPath(
      int[] cameFrom, int current, Translation2d exactGoal) {
    List<Translation2d> points = new ArrayList<>();
    points.add(exactGoal);
    int cursor = cameFrom[current];
    // Stop before startIdx (where cameFrom[cursor] == -1) so the robot never turns backward
    while (cursor != -1 && cameFrom[cursor] != -1) {
      int c = cursor % GRID_COLS;
      int r = cursor / GRID_COLS;
      points.add(new Translation2d(c * GRID_RES, r * GRID_RES));
      cursor = cameFrom[cursor];
    }
    Collections.reverse(points);
    return points;
  }

  private List<Translation2d> smoothPath(Translation2d start, List<Translation2d> rawPath) {
    if (rawPath.size() <= 1) {
      return rawPath;
    }
    List<Translation2d> smoothed = new ArrayList<>();
    Translation2d anchor = start;
    int idx = 0;

    while (idx < rawPath.size()) {
      int furthest = idx;
      for (int probe = rawPath.size() - 1; probe > idx; probe--) {
        Translation2d candidate = rawPath.get(probe);
        if (isSegmentClear(anchor, candidate) && !segmentCrossesBump(anchor, candidate)) {
          furthest = probe;
          break;
        }
      }
      Translation2d nextWaypoint = rawPath.get(furthest);
      smoothed.add(nextWaypoint);
      anchor = nextWaypoint;
      idx = furthest + 1;
    }
    return smoothed;
  }

  private int[] nearestWalkableCell(int col, int row) {
    if (isCellWalkable(col, row)) {
      return new int[] {col, row};
    }
    for (int radius = 1; radius <= 6; radius++) {
      int bestC = col;
      int bestR = row;
      double bestDist = Double.MAX_VALUE;
      for (int dc = -radius; dc <= radius; dc++) {
        for (int dr = -radius; dr <= radius; dr++) {
          int nc = clampCol(col + dc);
          int nr = clampRow(row + dr);
          if (isCellWalkable(nc, nr)) {
            double d = Math.hypot(dc, dr);
            if (d < bestDist) {
              bestDist = d;
              bestC = nc;
              bestR = nr;
            }
          }
        }
      }
      if (bestDist < Double.MAX_VALUE) {
        return new int[] {bestC, bestR};
      }
    }
    return new int[] {col, row};
  }

  private boolean isCellWalkable(int col, int row) {
    return isPositionClear(col * GRID_RES, row * GRID_RES);
  }

  private boolean isPositionClear(double x, double y) {
    if (x < CLEARANCE_RADIUS
        || x > Field2026Physics.FIELD_LENGTH - CLEARANCE_RADIUS
        || y < CLEARANCE_RADIUS
        || y > Field2026Physics.FIELD_WIDTH - CLEARANCE_RADIUS) {
      return false;
    }
    for (Field2026Physics.BoxCollider box : Field2026Physics.SOLID_OBSTACLES) {
      double closestX = Math.clamp(x, box.minX(), box.maxX());
      double closestY = Math.clamp(y, box.minY(), box.maxY());
      if (Math.hypot(x - closestX, y - closestY) < CLEARANCE_RADIUS) {
        return false;
      }
    }
    for (Field2026Physics.TrianglePrismCollider tri : Field2026Physics.TRIANGLE_SIDE_BLOCKERS) {
      Translation2d closest = tri.closestPoint2d(x, y);
      if (Math.hypot(x - closest.getX(), y - closest.getY()) < CLEARANCE_RADIUS) {
        return false;
      }
    }
    return true;
  }

  private boolean isSegmentClear(Translation2d a, Translation2d b) {
    double dist = a.getDistance(b);
    int samples = Math.max(2, (int) Math.ceil(dist / 0.10));
    for (int i = 1; i <= samples; i++) {
      double t = (double) i / samples;
      double x = a.getX() + t * (b.getX() - a.getX());
      double y = a.getY() + t * (b.getY() - a.getY());
      if (!isPositionClear(x, y)) {
        return false;
      }
    }
    return true;
  }

  private boolean segmentCrossesBump(Translation2d a, Translation2d b) {
    double dist = a.getDistance(b);
    int samples = Math.max(2, (int) Math.ceil(dist / 0.15));
    for (int i = 0; i <= samples; i++) {
      double t = (double) i / samples;
      double x = a.getX() + t * (b.getX() - a.getX());
      double y = a.getY() + t * (b.getY() - a.getY());
      if (Field2026Physics.getTerrainHeight(x, y) > 0.0) {
        return true;
      }
    }
    return false;
  }

  private int clampCol(int col) {
    return Math.clamp(col, 0, GRID_COLS - 1);
  }

  private int clampRow(int row) {
    return Math.clamp(row, 0, GRID_ROWS - 1);
  }

  private void publishPath(List<Translation2d> waypoints) {
    double[] flatPoses = new double[waypoints.size() * 3];
    for (int i = 0; i < waypoints.size(); i++) {
      flatPoses[i * 3] = waypoints.get(i).getX();
      flatPoses[i * 3 + 1] = waypoints.get(i).getY();
      flatPoses[i * 3 + 2] = 0.0;
    }
    autoTable.getEntry("PlannedPath").setDoubleArray(flatPoses);
  }

  private Translation2d getVisibleRedHubCenter() {
    double[] fiducials = cameraTable.getEntry("rawfiducials").getDoubleArray(new double[0]);
    if (fiducials.length >= 7) {
      for (int i = 0; i + 6 < fiducials.length; i += 7) {
        int tagId = (int) Math.round(fiducials[i]);
        if (RED_HUB_TAG_IDS.contains(tagId)) {
          return Field2026Physics.RED_HUB_CENTER;
        }
      }
    }

    boolean hasTarget = cameraTable.getEntry("tv").getDouble(0.0) == 1.0;
    int tid = (int) Math.round(cameraTable.getEntry("tid").getDouble(-1));
    if (hasTarget && RED_HUB_TAG_IDS.contains(tid)) {
      return Field2026Physics.RED_HUB_CENTER;
    }
    return null;
  }
}
