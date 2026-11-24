/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.simulation;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

public class FuelSim {
  private enum Mode {
    INTAKE,
    SHOOT,
    OUTTAKE
  }

  private enum FuelState {
    ON_FIELD,
    IN_FLIGHT,
    INTAKING,
    IN_HOPPER,
    FEEDING,
    SCORED_CHUTE
  }

  private static final class FuelPiece {
    // World-frame position & velocity (used when ON_FIELD, IN_FLIGHT, SCORED_CHUTE)
    double x, y, z;
    double vx, vy, vz;

    // Robot-local position & velocity (used when INTAKING, IN_HOPPER, FEEDING)
    double lx, ly, lz;
    double lvx, lvy, lvz;

    FuelState state;

    FuelPiece(double x, double y, double z, double vx, double vy, double vz, FuelState state) {
      this.x = x;
      this.y = y;
      this.z = z;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
      this.state = state;
    }

    static FuelPiece inHopper(double lx, double ly, double lz) {
      FuelPiece piece = new FuelPiece(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, FuelState.IN_HOPPER);
      piece.lx = lx;
      piece.ly = ly;
      piece.lz = lz;
      return piece;
    }

    boolean isInsideRobot() {
      return state == FuelState.INTAKING
          || state == FuelState.IN_HOPPER
          || state == FuelState.FEEDING;
    }

    Pose3d toWorldPose3d(Pose3d robot3d) {
      if (isInsideRobot()) {
        return robot3d.plus(new Transform3d(lx, ly, lz, Rotation3d.ZERO));
      }
      return new Pose3d(x, y, z, Rotation3d.ZERO);
    }
  }

  // Physical constants for 2026 Fuel (foam ball)
  private static final double FUEL_RADIUS = 0.075; // 15 cm (~5.9 in) diameter
  private static final double GRAVITY = 9.81;
  private static final double AIR_DRAG = 0.18;
  private static final double FLOOR_RESTITUTION = 0.62;
  private static final double WALL_RESTITUTION = 0.55;
  private static final double BALL_RESTITUTION = 0.65;
  private static final double ROLLING_FRICTION = 1.6;
  private static final double DT = 0.02;
  private static final int PHYSICS_SUBSTEPS = 4;
  private static final int COLLISION_ITERATIONS = 3;

  // Intake geometry & physics (robot-local meters)
  private static final double INTAKE_MOUTH_X = 0.46;
  private static final double INTAKE_ENTRY_X = 0.15;
  private static final double INTAKE_MOUTH_HALF_WIDTH = 0.27;
  private static final double INTAKE_SUCTION_MAX_X = 0.66;
  private static final double INTAKE_MAX_CAPTURE_Z = 0.28;
  private static final double INTAKE_RAMP_START_Z = FUEL_RADIUS;
  private static final double INTAKE_RAMP_END_Z = 0.235;
  private static final double INTAKE_BELT_MAX_SPEED = 2.6;

  // Hopper interior geometry & open-top overflow rim (robot-local meters)
  private static final double HOPPER_MIN_X = -0.31;
  private static final double HOPPER_MAX_X = 0.16;
  private static final double HOPPER_CENTER_X = 0.5 * (HOPPER_MIN_X + HOPPER_MAX_X);
  private static final double HOPPER_HALF_WIDTH = 0.235;
  private static final double HOPPER_REAR_FLOOR_Z = 0.36;
  private static final double HOPPER_FRONT_FLOOR_Z = 0.22;
  private static final double HOPPER_RIM_Z = 0.46;
  private static final double HOPPER_FLOOR_RESTITUTION = 0.30;
  private static final double HOPPER_WALL_RESTITUTION = 0.38;
  private static final double HOPPER_BALL_RESTITUTION = 0.42;
  private static final double HOPPER_ROLLING_FRICTION = 3.2;

  // Shooter & feeder parameters
  private static final double SHOT_COOLDOWN_SECONDS = 0.22;
  private static final double LAUNCH_PITCH_RAD = Math.toRadians(68.0);
  private static final double WHEEL_RADIUS_METERS = 0.0508; // 2-inch radius launcher wheel
  private static final double SURFACE_TRANSFER_FACTOR = 0.28;
  private static final double MIN_SHOOT_RAD_PER_SEC =
      505.0; // ~4820 RPM (~95% of 0.9-throttle NEO speed)
  private static final double SPINUP_READY_RATIO = 0.95;
  private static final double FEEDER_CHUTE_SPEED = 3.2;
  private static final Transform3d SHOT_ORIGIN = new Transform3d(0.24, 0.0, 0.52, Rotation3d.ZERO);

  private static final TelemetryTable table = Telemetry.getTable("FuelSim");
  private static final List<FuelPiece> activeFuel = new ArrayList<>();
  private static final Random random = new Random(2026);

  private static Mode mode = null;
  private static boolean isPaused = false;
  private static double shotCooldownTimer = 0.0;
  private static int blueScore = 0;
  private static int redScore = 0;

  // Robot kinematics tracking for inertial & angular transfer
  private static Pose3d prevRobotPose3d = null;
  private static Translation2d prevRobotVel = Translation2d.ZERO;
  private static Translation3d robotAccelLocal = Translation3d.ZERO;
  private static double robotOmegaZ = 0.0;

  static DoubleSupplier feederVoltsSupplier = () -> 0.0;
  static DoubleSupplier intakeLauncherVoltsSupplier = () -> 0.0;
  static DoubleSupplier launcherRadPerSecSupplier = () -> 0.0;
  static Supplier<Pose2d> robotPoseSupplier = () -> Pose2d.ZERO;
  static Supplier<Pose3d> robotPose3dSupplier = () -> new Pose3d(robotPoseSupplier.get());
  static Supplier<Translation2d> robotVelocitySupplier = () -> Translation2d.ZERO;

  static {
    resetFieldFuel();
  }

  public static void setMotorSuppliers(
      DoubleSupplier intakeLauncherSupplier, DoubleSupplier feederSupplier) {
    intakeLauncherVoltsSupplier = intakeLauncherSupplier;
    feederVoltsSupplier = feederSupplier;
  }

  /** Returns the total number of Fuel balls currently inside the intake, hopper, or feeder. */
  public static int getStoredFuelCount() {
    int count = 0;
    for (FuelPiece fuel : activeFuel) {
      if (fuel.isInsideRobot()) {
        count++;
      }
    }
    return count;
  }

  /** Returns the number of Fuel balls currently climbing the intake ramp into the hopper. */
  public static int getIntakingFuelCount() {
    int count = 0;
    for (FuelPiece fuel : activeFuel) {
      if (fuel.state == FuelState.INTAKING) {
        count++;
      }
    }
    return count;
  }

  /** Returns field coordinates of all loose Fuel balls available to be intaken. */
  public static List<Translation2d> getCollectibleFieldFuelPositions() {
    List<Translation2d> positions = new ArrayList<>();
    for (FuelPiece fuel : activeFuel) {
      if (fuel.state == FuelState.ON_FIELD || fuel.state == FuelState.IN_FLIGHT) {
        positions.add(new Translation2d(fuel.x, fuel.y));
      }
    }
    return positions;
  }

  /** Spawns initial Fuel clusters in the Neutral Zone, Depots, and preloaded robot hopper. */
  public static void resetFieldFuel() {
    activeFuel.clear();
    blueScore = 0;
    redScore = 0;
    shotCooldownTimer = 0.0;
    prevRobotPose3d = null;
    prevRobotVel = Translation2d.ZERO;
    robotAccelLocal = Translation3d.ZERO;
    robotOmegaZ = 0.0;

    // Preload 3 physical Fuel balls into the front row of the hopper
    activeFuel.add(FuelPiece.inHopper(0.06, -0.15, getHopperFloorZ(0.06)));
    activeFuel.add(FuelPiece.inHopper(0.06, 0.00, getHopperFloorZ(0.06)));
    activeFuel.add(FuelPiece.inHopper(0.06, 0.15, getHopperFloorZ(0.06)));

    // Center Neutral Zone Fuel grid (4 columns x 6 rows = 24 Fuel)
    for (int col = 0; col < 4; col++) {
      for (int row = 0; row < 6; row++) {
        double x = 7.67 + col * 0.40;
        double y = 1.535 + row * 1.00;
        activeFuel.add(
            new FuelPiece(
                x,
                y,
                Field2026Physics.getTerrainHeight(x, y) + FUEL_RADIUS,
                0,
                0,
                0,
                FuelState.ON_FIELD));
      }
    }

    // Blue & Red Alliance Depot Fuel clusters
    for (int i = 0; i < 6; i++) {
      double y = 2.535 + i * 0.60;
      activeFuel.add(new FuelPiece(1.35, y, FUEL_RADIUS, 0, 0, 0, FuelState.ON_FIELD));
      activeFuel.add(
          new FuelPiece(
              Field2026Physics.FIELD_LENGTH - 1.35, y, FUEL_RADIUS, 0, 0, 0, FuelState.ON_FIELD));
    }
  }

  /** Updates the fuel sim every 20ms loop. */
  public static void periodic() {
    shotCooldownTimer = Math.max(0.0, shotCooldownTimer - DT);
    updateRobotKinematics();
    updateMode();
    handleRobotInteractions();
    stepFuelPhysics();
    updateVisualization();
  }

  private static Translation3d rotateWorldToLocal(Translation3d worldVec, Rotation3d rot) {
    // Invert intrinsic Z-Y-X rotation (yaw -> pitch -> roll) by applying -yaw, -pitch, -roll
    return worldVec
        .rotateBy(new Rotation3d(0.0, 0.0, -rot.getZ()))
        .rotateBy(new Rotation3d(0.0, -rot.getY(), 0.0))
        .rotateBy(new Rotation3d(-rot.getX(), 0.0, 0.0));
  }

  private static void updateRobotKinematics() {
    Pose3d robot3d = robotPose3dSupplier.get();
    Translation2d robotVel = robotVelocitySupplier.get();

    if (prevRobotPose3d != null) {
      double dYaw =
          MathUtil.angleModulus(
              robot3d.getRotation().getZ() - prevRobotPose3d.getRotation().getZ());
      robotOmegaZ = dYaw / DT;

      Translation2d accelWorld2d = robotVel.minus(prevRobotVel).div(DT);
      double ax = Math.clamp(accelWorld2d.getX(), -15.0, 15.0);
      double ay = Math.clamp(accelWorld2d.getY(), -15.0, 15.0);
      robotAccelLocal = rotateWorldToLocal(new Translation3d(ax, ay, 0.0), robot3d.getRotation());
    } else {
      robotOmegaZ = 0.0;
      robotAccelLocal = Translation3d.ZERO;
    }

    prevRobotPose3d = robot3d;
    prevRobotVel = robotVel;
  }

  private static void updateMode() {
    double intakeLauncherVolts = intakeLauncherVoltsSupplier.getAsDouble();
    double feederVolts = feederVoltsSupplier.getAsDouble();
    isPaused = false;
    if (intakeLauncherVolts > 0.5 && feederVolts > 0.5) {
      mode = Mode.SHOOT;
    } else if (intakeLauncherVolts < -0.5 && feederVolts > 0.5) {
      mode = Mode.OUTTAKE;
    } else if (intakeLauncherVolts > 0.5 && feederVolts < -0.5) {
      mode = Mode.INTAKE;
    } else {
      isPaused = true;
    }
  }

  private static boolean hasFeedingBall() {
    for (FuelPiece fuel : activeFuel) {
      if (fuel.state == FuelState.FEEDING) {
        return true;
      }
    }
    return false;
  }

  private static boolean isLauncherSpunUp() {
    double volts = Math.abs(intakeLauncherVoltsSupplier.getAsDouble());
    double currentRadPerSec = Math.abs(launcherRadPerSecSupplier.getAsDouble());
    // Require at least 95% of the NEO's steady-state speed for the applied voltage
    double targetRadPerSec = Math.max(MIN_SHOOT_RAD_PER_SEC, volts * 49.5 * SPINUP_READY_RATIO);
    return currentRadPerSec >= targetRadPerSec;
  }

  private static void handleRobotInteractions() {
    Pose2d robot2d = robotPoseSupplier.get();
    Pose3d robot3d = robotPose3dSupplier.get();
    Translation2d robotVel = robotVelocitySupplier.get();
    boolean intakingActive = !isPaused && mode == Mode.INTAKE;

    // 1. Trigger feeder transfer from hopper into shooter chute once flywheel is fully spun up
    if (!isPaused
        && mode == Mode.SHOOT
        && shotCooldownTimer <= 0.0
        && !hasFeedingBall()
        && isLauncherSpunUp()) {
      FuelPiece nextToFeed = selectNextHopperBallForFeeder();
      if (nextToFeed != null) {
        nextToFeed.state = FuelState.FEEDING;
        shotCooldownTimer = SHOT_COOLDOWN_SECONDS;
      }
    }

    // 2. Intake capture & bumper collision resolution for external field balls
    double minBumperDist = Field2026Physics.ROBOT_COLLISION_RADIUS + FUEL_RADIUS;

    for (FuelPiece fuel : activeFuel) {
      if (fuel.isInsideRobot() || fuel.state == FuelState.SCORED_CHUTE) {
        continue;
      }

      Translation3d localPos = worldToRobotLocalPos(fuel, robot3d);
      boolean inIntakeMouthCorridor =
          localPos.getX() >= 0.20
              && localPos.getX() <= INTAKE_SUCTION_MAX_X
              && Math.abs(localPos.getY()) <= INTAKE_MOUTH_HALF_WIDTH + 0.05
              && localPos.getZ() <= INTAKE_MAX_CAPTURE_Z;

      if (intakingActive && inIntakeMouthCorridor) {
        // Capture ball into physical intake ramp once it crosses the roller throat
        if (localPos.getX() <= INTAKE_MOUTH_X + 0.04
            && Math.abs(localPos.getY()) <= INTAKE_MOUTH_HALF_WIDTH) {
          captureFuelIntoIntake(fuel, robot3d, robotVel);
          continue;
        }

        // Apply inward roller suction force to draw balls in the approach zone into the throat
        Translation3d targetWorld =
            robot3d
                .plus(
                    new Transform3d(
                        INTAKE_MOUTH_X - 0.04, 0.0, INTAKE_RAMP_START_Z, Rotation3d.ZERO))
                .getTranslation();
        double pullDx = targetWorld.getX() - fuel.x;
        double pullDy = targetWorld.getY() - fuel.y;
        double pullDist = Math.hypot(pullDx, pullDy);
        if (pullDist > 1e-4) {
          double pullSpeed = 2.6;
          fuel.vx += ((pullDx / pullDist) * pullSpeed - (fuel.vx - robotVel.getX())) * 0.35;
          fuel.vy += ((pullDy / pullDist) * pullSpeed - (fuel.vy - robotVel.getY())) * 0.35;
          fuel.state = FuelState.IN_FLIGHT;
        }
        continue;
      }

      // Push ground-level Fuel out of the robot's bumper footprint when not intaking
      if (fuel.z <= robot3d.getZ() + 0.45) {
        double dx = fuel.x - robot2d.getX();
        double dy = fuel.y - robot2d.getY();
        double dist = Math.hypot(dx, dy);
        if (dist < minBumperDist) {
          double nx = dist > 1e-6 ? dx / dist : Math.cos(robot2d.getRotation().getRadians());
          double ny = dist > 1e-6 ? dy / dist : Math.sin(robot2d.getRotation().getRadians());
          fuel.x = robot2d.getX() + nx * minBumperDist;
          fuel.y = robot2d.getY() + ny * minBumperDist;
          fuel.vx = robotVel.getX() + nx * 1.4;
          fuel.vy = robotVel.getY() + ny * 1.4;
          fuel.state = FuelState.IN_FLIGHT;
        }
      }
    }
  }

  private static FuelPiece selectNextHopperBallForFeeder() {
    FuelPiece best = null;
    double bestScore = -Double.MAX_VALUE;
    for (FuelPiece fuel : activeFuel) {
      if (fuel.state == FuelState.IN_HOPPER) {
        // Favor balls furthest forward (+lx), centered near the feeder throat (ly ~ 0), and on the
        // bottom layer (-lz)
        double score = fuel.lx - 0.65 * Math.abs(fuel.ly) - 0.85 * fuel.lz;
        if (score > bestScore) {
          bestScore = score;
          best = fuel;
        }
      }
    }
    return best;
  }

  private static Translation3d worldToRobotLocalPos(FuelPiece fuel, Pose3d robot3d) {
    return rotateWorldToLocal(
        new Translation3d(fuel.x, fuel.y, fuel.z).minus(robot3d.getTranslation()),
        robot3d.getRotation());
  }

  private static void captureFuelIntoIntake(
      FuelPiece fuel, Pose3d robot3d, Translation2d robotVel) {
    Translation3d localPos = worldToRobotLocalPos(fuel, robot3d);
    Translation3d worldRelVel =
        new Translation3d(fuel.vx - robotVel.getX(), fuel.vy - robotVel.getY(), fuel.vz);
    Translation3d localVel = rotateWorldToLocal(worldRelVel, robot3d.getRotation());

    fuel.lx = Math.clamp(localPos.getX(), INTAKE_ENTRY_X + 0.02, INTAKE_MOUTH_X);
    fuel.ly = Math.clamp(localPos.getY(), -INTAKE_MOUTH_HALF_WIDTH, INTAKE_MOUTH_HALF_WIDTH);
    fuel.lz = Math.max(getIntakeRampFloorZ(fuel.lx), localPos.getZ());

    // Subtract rotational frame velocity (omega x r)
    fuel.lvx = localVel.getX() + robotOmegaZ * fuel.ly;
    fuel.lvy = localVel.getY() - robotOmegaZ * fuel.lx;
    fuel.lvz = localVel.getZ();
    fuel.state = FuelState.INTAKING;
  }

  private static void releaseFuelToWorld(FuelPiece fuel, Pose3d robot3d, Translation2d robotVel) {
    Pose3d worldPose = robot3d.plus(new Transform3d(fuel.lx, fuel.ly, fuel.lz, Rotation3d.ZERO));
    Translation3d surfaceLocalVel =
        new Translation3d(-robotOmegaZ * fuel.ly, robotOmegaZ * fuel.lx, 0.0);
    Translation3d totalLocalVel =
        new Translation3d(fuel.lvx, fuel.lvy, fuel.lvz).plus(surfaceLocalVel);
    Translation3d worldVel =
        new Translation3d(robotVel.getX(), robotVel.getY(), 0.0)
            .plus(totalLocalVel.rotateBy(robot3d.getRotation()));

    fuel.x = worldPose.getX();
    fuel.y = worldPose.getY();
    fuel.z = worldPose.getZ();
    fuel.vx = worldVel.getX();
    fuel.vy = worldVel.getY();
    fuel.vz = worldVel.getZ();
    fuel.state = FuelState.IN_FLIGHT;
  }

  private static double getIntakeRampFloorZ(double lx) {
    double t = Math.clamp((INTAKE_MOUTH_X - lx) / (INTAKE_MOUTH_X - INTAKE_ENTRY_X), 0.0, 1.0);
    return INTAKE_RAMP_START_Z + t * (INTAKE_RAMP_END_Z - INTAKE_RAMP_START_Z);
  }

  private static double getHopperFloorZ(double lx) {
    double t = Math.clamp((HOPPER_MAX_X - lx) / (HOPPER_MAX_X - HOPPER_MIN_X), 0.0, 1.0);
    return HOPPER_FRONT_FLOOR_Z + t * (HOPPER_REAR_FLOOR_Z - HOPPER_FRONT_FLOOR_Z);
  }

  private static double computeLaunchSpeed() {
    double radPerSec = Math.abs(launcherRadPerSecSupplier.getAsDouble());
    double tangentialSpeed = radPerSec * WHEEL_RADIUS_METERS * SURFACE_TRANSFER_FACTOR;
    return Math.max(6.2, tangentialSpeed);
  }

  private static void stepFuelPhysics() {
    double subDt = DT / PHYSICS_SUBSTEPS;
    Pose3d robot3d = robotPose3dSupplier.get();
    Translation2d robotVel = robotVelocitySupplier.get();

    // Gravity vector expressed in the robot's 3D pitched/rolled frame
    Translation3d localGravity =
        rotateWorldToLocal(new Translation3d(0.0, 0.0, -GRAVITY), robot3d.getRotation());

    double intakeRollerPower =
        Math.clamp(Math.abs(intakeLauncherVoltsSupplier.getAsDouble()) / 12.0, 0.0, 1.0);
    double feederPower = Math.clamp(Math.abs(feederVoltsSupplier.getAsDouble()) / 12.0, 0.0, 1.0);

    for (int step = 0; step < PHYSICS_SUBSTEPS; step++) {
      for (FuelPiece fuel : activeFuel) {
        if (fuel.state == FuelState.INTAKING) {
          stepIntakeRampPhysics(fuel, subDt, localGravity, intakeRollerPower, robot3d, robotVel);
          continue;
        }
        if (fuel.state == FuelState.IN_HOPPER) {
          stepHopperPhysics(fuel, subDt, localGravity, feederPower, robot3d, robotVel);
          continue;
        }
        if (fuel.state == FuelState.FEEDING) {
          stepFeederChutePhysics(fuel, subDt, feederPower, robot3d, robotVel);
          continue;
        }

        if (fuel.state == FuelState.ON_FIELD) {
          double slopeX = Field2026Physics.getBumpSlopeX(fuel.x, fuel.y);
          if (Math.abs(slopeX) > 1e-4) {
            fuel.vx += -GRAVITY * slopeX * subDt;
            fuel.state = FuelState.IN_FLIGHT;
          } else {
            fuel.z = Field2026Physics.getTerrainHeight(fuel.x, fuel.y) + FUEL_RADIUS;
            continue;
          }
        }

        double prevZ = fuel.z;

        // Aerodynamic drag & gravity
        double speed = Math.sqrt(fuel.vx * fuel.vx + fuel.vy * fuel.vy + fuel.vz * fuel.vz);
        double dragFactor = AIR_DRAG * speed;
        fuel.vx -= dragFactor * fuel.vx * subDt;
        fuel.vy -= dragFactor * fuel.vy * subDt;
        fuel.vz -= (GRAVITY + dragFactor * fuel.vz) * subDt;

        fuel.x += fuel.vx * subDt;
        fuel.y += fuel.vy * subDt;
        fuel.z += fuel.vz * subDt;

        // Check Hub funnel scoring when descending into the top opening
        if (fuel.state == FuelState.IN_FLIGHT
            && fuel.vz < 0.0
            && prevZ >= Field2026Physics.HUB_FUNNEL_HEIGHT - FUEL_RADIUS
            && fuel.z < Field2026Physics.HUB_FUNNEL_HEIGHT) {
          if (isInsideHubFunnel(fuel.x, fuel.y, Field2026Physics.BLUE_HUB_CENTER)) {
            blueScore++;
            fuel.state = FuelState.SCORED_CHUTE;
          } else if (isInsideHubFunnel(fuel.x, fuel.y, Field2026Physics.RED_HUB_CENTER)) {
            redScore++;
            fuel.state = FuelState.SCORED_CHUTE;
          }
        }

        // Handle ball inside Hub return chute
        if (fuel.state == FuelState.SCORED_CHUTE) {
          boolean isBlue = fuel.x < Field2026Physics.FIELD_LENGTH / 2.0;
          Translation2d hub =
              isBlue ? Field2026Physics.BLUE_HUB_CENTER : Field2026Physics.RED_HUB_CENTER;

          // Center ball smoothly inside the chute while dropping
          fuel.vx = (hub.getX() - fuel.x) * 4.0;
          fuel.vy = (hub.getY() - fuel.y) * 4.0;

          if (fuel.z <= 0.45) {
            // Eject scored Fuel cleanly outside the square Hub collider back into active play
            double angle = (random.nextDouble() - 0.5) * Math.PI * 0.8 + (isBlue ? 0.0 : Math.PI);
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);
            double squareScale = 1.0 / Math.max(Math.abs(cosA), Math.abs(sinA));
            double exitDist = (Field2026Physics.HUB_HALF_SIZE + FUEL_RADIUS + 0.08) * squareScale;

            fuel.x = hub.getX() + cosA * exitDist;
            fuel.y = hub.getY() + sinA * exitDist;
            fuel.vx = cosA * 2.4;
            fuel.vy = sinA * 2.4;
            fuel.vz = 0.5;
            fuel.state = FuelState.IN_FLIGHT;
          }
          continue;
        }

        // Perimeter wall collisions
        if (fuel.x < FUEL_RADIUS) {
          fuel.x = FUEL_RADIUS;
          fuel.vx = -fuel.vx * WALL_RESTITUTION;
        } else if (fuel.x > Field2026Physics.FIELD_LENGTH - FUEL_RADIUS) {
          fuel.x = Field2026Physics.FIELD_LENGTH - FUEL_RADIUS;
          fuel.vx = -fuel.vx * WALL_RESTITUTION;
        }

        if (fuel.y < FUEL_RADIUS) {
          fuel.y = FUEL_RADIUS;
          fuel.vy = -fuel.vy * WALL_RESTITUTION;
        } else if (fuel.y > Field2026Physics.FIELD_WIDTH - FUEL_RADIUS) {
          fuel.y = Field2026Physics.FIELD_WIDTH - FUEL_RADIUS;
          fuel.vy = -fuel.vy * WALL_RESTITUTION;
        }

        // Solid Hub / Blocker / Triangle Side Blocker / Trench obstacle collisions
        resolveFuelObstacleBounce(fuel, prevZ);

        // Floor / Bump terrain bounce
        double floorZ = Field2026Physics.getTerrainHeight(fuel.x, fuel.y) + FUEL_RADIUS;
        if (fuel.z <= floorZ) {
          fuel.z = floorZ;
          if (fuel.vz < -0.65) {
            fuel.vz = -fuel.vz * FLOOR_RESTITUTION;
            fuel.vx *= 0.85;
            fuel.vy *= 0.85;
          } else {
            fuel.vz = 0.0;
            double slopeX = Field2026Physics.getBumpSlopeX(fuel.x, fuel.y);
            fuel.vx += -GRAVITY * slopeX * subDt;
            fuel.vx *= Math.max(0.0, 1.0 - ROLLING_FRICTION * subDt);
            fuel.vy *= Math.max(0.0, 1.0 - ROLLING_FRICTION * subDt);
            if (Math.abs(slopeX) <= 1e-4 && Math.hypot(fuel.vx, fuel.vy) < 0.08) {
              fuel.vx = 0.0;
              fuel.vy = 0.0;
              fuel.state = FuelState.ON_FIELD;
            }
          }
        }
      }

      for (int iter = 0; iter < COLLISION_ITERATIONS; iter++) {
        resolveInRobotBallCollisions();
        resolveFuelToFuelCollisions();
      }
    }
  }

  private static void stepIntakeRampPhysics(
      FuelPiece fuel,
      double subDt,
      Translation3d localGravity,
      double intakeRollerPower,
      Pose3d robot3d,
      Translation2d robotVel) {
    // Apply 3D local gravity and chassis inertial acceleration
    fuel.lvx += (localGravity.getX() - robotAccelLocal.getX()) * subDt;
    fuel.lvy += (localGravity.getY() - robotAccelLocal.getY()) * subDt;
    fuel.lvz += localGravity.getZ() * subDt;

    double rampSlopeAngle =
        Math.atan2(INTAKE_RAMP_END_Z - INTAKE_RAMP_START_Z, INTAKE_MOUTH_X - INTAKE_ENTRY_X);

    if (!isPaused && mode == Mode.INTAKE) {
      // Roller traction pulls ball up the intake ramp and centers it laterally
      double targetBeltSpeed = INTAKE_BELT_MAX_SPEED * Math.max(0.35, intakeRollerPower);
      double targetVx = -targetBeltSpeed * Math.cos(rampSlopeAngle);
      double targetVz = targetBeltSpeed * Math.sin(rampSlopeAngle);
      fuel.lvx += (targetVx - fuel.lvx) * Math.min(1.0, 14.0 * subDt);
      fuel.lvz += (targetVz - fuel.lvz) * Math.min(1.0, 14.0 * subDt);
      fuel.lvy += (-fuel.ly * 4.5 - fuel.lvy) * Math.min(1.0, 10.0 * subDt);
    } else if (!isPaused && mode == Mode.OUTTAKE) {
      // Reversed intake rollers eject the ball forward out the mouth
      double ejectSpeed = INTAKE_BELT_MAX_SPEED * Math.max(0.45, intakeRollerPower);
      fuel.lvx += (ejectSpeed * Math.cos(rampSlopeAngle) - fuel.lvx) * Math.min(1.0, 16.0 * subDt);
      fuel.lvz += (-ejectSpeed * Math.sin(rampSlopeAngle) - fuel.lvz) * Math.min(1.0, 16.0 * subDt);
    } else {
      // Motors stopped mid-intake: gravity slides ball down the ramp toward the mouth
      fuel.lvx += GRAVITY * Math.sin(rampSlopeAngle) * Math.cos(rampSlopeAngle) * subDt;
      fuel.lvx *= Math.max(0.0, 1.0 - 2.2 * subDt);
      fuel.lvy *= Math.max(0.0, 1.0 - 2.2 * subDt);
    }

    fuel.lx += fuel.lvx * subDt;
    fuel.ly += fuel.lvy * subDt;
    fuel.lz += fuel.lvz * subDt;

    // Intake ramp floor & side guide rail constraints
    double rampFloorZ = getIntakeRampFloorZ(fuel.lx);
    if (fuel.lz <= rampFloorZ) {
      fuel.lz = rampFloorZ;
      if (fuel.lvz < 0.0) {
        fuel.lvz = -fuel.lvz * 0.20;
      }
    }

    double t = Math.clamp((INTAKE_MOUTH_X - fuel.lx) / (INTAKE_MOUTH_X - INTAKE_ENTRY_X), 0.0, 1.0);
    double halfRailWidth =
        INTAKE_MOUTH_HALF_WIDTH + t * (HOPPER_HALF_WIDTH - INTAKE_MOUTH_HALF_WIDTH);
    if (fuel.ly < -halfRailWidth) {
      fuel.ly = -halfRailWidth;
      fuel.lvy = Math.abs(fuel.lvy) * 0.35;
    } else if (fuel.ly > halfRailWidth) {
      fuel.ly = halfRailWidth;
      fuel.lvy = -Math.abs(fuel.lvy) * 0.35;
    }

    // Transition into hopper once past the intake crest, or back onto field if rolled/ejected out
    if (fuel.lx <= INTAKE_ENTRY_X) {
      fuel.state = FuelState.IN_HOPPER;
    } else if (fuel.lx > INTAKE_MOUTH_X + 0.06) {
      releaseFuelToWorld(fuel, robot3d, robotVel);
    }
  }

  private static void stepHopperPhysics(
      FuelPiece fuel,
      double subDt,
      Translation3d localGravity,
      double feederPower,
      Pose3d robot3d,
      Translation2d robotVel) {
    // 3D local gravity + robot chassis inertial acceleration (slosh under accel/braking/turning)
    fuel.lvx += (localGravity.getX() - robotAccelLocal.getX()) * subDt;
    fuel.lvy += (localGravity.getY() - robotAccelLocal.getY()) * subDt;
    fuel.lvz += localGravity.getZ() * subDt;

    // Feeder roller agitation & outtake reversal inside the hopper
    if (!isPaused && mode == Mode.SHOOT) {
      // Funnel front-half balls gently toward the centered feeder throat at (0.14, 0.0)
      if (fuel.lx > -0.10) {
        double pullScale = Math.max(0.4, feederPower);
        fuel.lvx += 2.2 * pullScale * subDt;
        fuel.lvy += -fuel.ly * 5.0 * pullScale * subDt;
      }
    } else if (!isPaused && mode == Mode.OUTTAKE) {
      // Drive balls forward and out into the reversed intake ramp
      double outScale = Math.max(0.5, feederPower);
      fuel.lvx += 4.5 * outScale * subDt;
      fuel.lvy += -fuel.ly * 4.0 * outScale * subDt;
      if (fuel.lx >= HOPPER_MAX_X - 0.01
          && Math.abs(fuel.ly) <= HOPPER_HALF_WIDTH
          && fuel.lz < HOPPER_RIM_Z) {
        fuel.lx = INTAKE_ENTRY_X + 0.02;
        fuel.lz = getIntakeRampFloorZ(fuel.lx);
        fuel.lvx = 2.0 * outScale;
        fuel.state = FuelState.INTAKING;
        return;
      }
    }

    fuel.lx += fuel.lvx * subDt;
    fuel.ly += fuel.lvy * subDt;
    fuel.lz += fuel.lvz * subDt;

    // Sloped hopper floor contact & downhill feed acceleration
    double floorZ = getHopperFloorZ(fuel.lx);
    if (fuel.lz <= floorZ) {
      fuel.lz = floorZ;
      if (fuel.lvz < -0.45) {
        fuel.lvz = -fuel.lvz * HOPPER_FLOOR_RESTITUTION;
      } else {
        fuel.lvz = 0.0;
        double floorSlopeAngle =
            Math.atan2(HOPPER_REAR_FLOOR_Z - HOPPER_FRONT_FLOOR_Z, HOPPER_MAX_X - HOPPER_MIN_X);
        fuel.lvx += GRAVITY * Math.sin(floorSlopeAngle) * Math.cos(floorSlopeAngle) * subDt;
        fuel.lvx *= Math.max(0.0, 1.0 - HOPPER_ROLLING_FRICTION * subDt);
        fuel.lvy *= Math.max(0.0, 1.0 - HOPPER_ROLLING_FRICTION * subDt);
      }
    }

    // Side/end walls contain balls below HOPPER_RIM_Z; balls pushed above the rim overflow outward
    if (fuel.lz < HOPPER_RIM_Z) {
      if (fuel.lx < HOPPER_MIN_X) {
        fuel.lx = HOPPER_MIN_X;
        fuel.lvx = Math.abs(fuel.lvx) * HOPPER_WALL_RESTITUTION;
      } else if (fuel.lx > HOPPER_MAX_X) {
        fuel.lx = HOPPER_MAX_X;
        fuel.lvx = -Math.abs(fuel.lvx) * HOPPER_WALL_RESTITUTION;
      }

      if (fuel.ly < -HOPPER_HALF_WIDTH) {
        fuel.ly = -HOPPER_HALF_WIDTH;
        fuel.lvy = Math.abs(fuel.lvy) * HOPPER_WALL_RESTITUTION;
      } else if (fuel.ly > HOPPER_HALF_WIDTH) {
        fuel.ly = HOPPER_HALF_WIDTH;
        fuel.lvy = -Math.abs(fuel.lvy) * HOPPER_WALL_RESTITUTION;
      }
    } else {
      // Encourage balls heaped above the open rim to roll outward toward the nearest edge
      double outX = fuel.lx - HOPPER_CENTER_X;
      double outY = fuel.ly;
      double outDist = Math.hypot(outX, outY);
      if (outDist > 1e-4) {
        fuel.lvx += (outX / outDist) * 2.8 * subDt;
        fuel.lvy += (outY / outDist) * 2.8 * subDt;
      } else {
        fuel.lvy += (random.nextBoolean() ? 1.5 : -1.5) * subDt;
      }

      // Spill over the rim into world physics once past the hopper wall perimeter
      if (fuel.lx < HOPPER_MIN_X - 0.03
          || fuel.lx > HOPPER_MAX_X + 0.03
          || Math.abs(fuel.ly) > HOPPER_HALF_WIDTH + 0.03) {
        if (outDist > 1e-4) {
          fuel.lvx += (outX / outDist) * 1.1;
          fuel.lvy += (outY / outDist) * 1.1;
        }
        releaseFuelToWorld(fuel, robot3d, robotVel);
      }
    }
  }

  private static void stepFeederChutePhysics(
      FuelPiece fuel, double subDt, double feederPower, Pose3d robot3d, Translation2d robotVel) {
    // If shoot button is released mid-feed before reaching the flywheel, drop back into hopper
    if (isPaused || mode != Mode.SHOOT) {
      fuel.lvz -= GRAVITY * subDt;
      fuel.lz += fuel.lvz * subDt;
      if (fuel.lz <= getHopperFloorZ(fuel.lx) + 0.02) {
        fuel.lx = Math.min(fuel.lx, HOPPER_MAX_X - 0.02);
        fuel.state = FuelState.IN_HOPPER;
      }
      return;
    }

    // Drive the ball smoothly up the feeder chute toward SHOT_ORIGIN (0.24, 0.0, 0.52)
    double targetX = SHOT_ORIGIN.getX();
    double targetY = SHOT_ORIGIN.getY();
    double targetZ = SHOT_ORIGIN.getZ();

    double dx = targetX - fuel.lx;
    double dy = targetY - fuel.ly;
    double dz = targetZ - fuel.lz;
    double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    double chuteSpeed = FEEDER_CHUTE_SPEED * Math.max(0.5, feederPower);

    if (dist > 0.025) {
      fuel.lvx = (dx / dist) * chuteSpeed;
      fuel.lvy = (dy / dist) * chuteSpeed;
      fuel.lvz = (dz / dist) * chuteSpeed;
      fuel.lx += fuel.lvx * subDt;
      fuel.ly += fuel.lvy * subDt;
      fuel.lz += fuel.lvz * subDt;
    } else {
      // Ball reached the flywheel muzzle: launch into world space
      fuel.lx = targetX;
      fuel.ly = targetY;
      fuel.lz = targetZ;

      double launchSpeed = computeLaunchSpeed();
      fuel.lvx = launchSpeed * Math.cos(LAUNCH_PITCH_RAD);
      fuel.lvy = 0.0;
      fuel.lvz = launchSpeed * Math.sin(LAUNCH_PITCH_RAD);
      releaseFuelToWorld(fuel, robot3d, robotVel);
    }
  }

  private static void resolveInRobotBallCollisions() {
    double minDist = 2.0 * FUEL_RADIUS;
    double minDistSq = minDist * minDist;

    for (int i = 0; i < activeFuel.size(); i++) {
      FuelPiece a = activeFuel.get(i);
      if (a.state != FuelState.INTAKING && a.state != FuelState.IN_HOPPER) {
        continue;
      }
      for (int j = i + 1; j < activeFuel.size(); j++) {
        FuelPiece b = activeFuel.get(j);
        if (b.state != FuelState.INTAKING && b.state != FuelState.IN_HOPPER) {
          continue;
        }

        double dx = b.lx - a.lx;
        double dy = b.ly - a.ly;
        double dz = b.lz - a.lz;
        double distSq = dx * dx + dy * dy + dz * dz;

        if (distSq < minDistSq) {
          double dist = Math.sqrt(distSq);
          double nx = dist > 1e-6 ? dx / dist : -1.0;
          double ny = dist > 1e-6 ? dy / dist : 0.0;
          double nz = dist > 1e-6 ? dz / dist : 0.0;

          double overlap = 0.5 * (minDist - dist);
          applyInRobotSeparation(a, -nx * overlap, -ny * overlap, -nz * overlap);
          applyInRobotSeparation(b, nx * overlap, ny * overlap, nz * overlap);

          double rvx = b.lvx - a.lvx;
          double rvy = b.lvy - a.lvy;
          double rvz = b.lvz - a.lvz;
          double velAlongNormal = rvx * nx + rvy * ny + rvz * nz;

          if (velAlongNormal < 0.0) {
            double impulse = -0.5 * (1.0 + HOPPER_BALL_RESTITUTION) * velAlongNormal;
            a.lvx -= impulse * nx;
            a.lvy -= impulse * ny;
            a.lvz -= impulse * nz;
            b.lvx += impulse * nx;
            b.lvy += impulse * ny;
            b.lvz += impulse * nz;
          }
        }
      }
    }
  }

  private static void applyInRobotSeparation(FuelPiece piece, double dx, double dy, double dz) {
    piece.lx += dx;
    piece.ly += dy;
    piece.lz = Math.max(getInRobotMinFloorZ(piece), piece.lz + dz);

    // Only clamp X/Y inside the hopper walls when below the open rim so top balls can overflow out
    if (piece.state == FuelState.INTAKING) {
      piece.lx = Math.clamp(piece.lx, INTAKE_ENTRY_X, INTAKE_MOUTH_X);
      piece.ly = Math.clamp(piece.ly, -INTAKE_MOUTH_HALF_WIDTH, INTAKE_MOUTH_HALF_WIDTH);
    } else if (piece.lz < HOPPER_RIM_Z) {
      piece.lx = Math.clamp(piece.lx, HOPPER_MIN_X, HOPPER_MAX_X);
      piece.ly = Math.clamp(piece.ly, -HOPPER_HALF_WIDTH, HOPPER_HALF_WIDTH);
    }
  }

  private static double getInRobotMinFloorZ(FuelPiece fuel) {
    return fuel.state == FuelState.INTAKING
        ? getIntakeRampFloorZ(fuel.lx)
        : getHopperFloorZ(fuel.lx);
  }

  private static boolean isInsideHubFunnel(double x, double y, Translation2d hubCenter) {
    return Math.hypot(x - hubCenter.getX(), y - hubCenter.getY())
        <= Field2026Physics.HUB_FUNNEL_RADIUS;
  }

  private static void resolveFuelObstacleBounce(FuelPiece fuel, double prevZ) {
    if (fuel.state == FuelState.SCORED_CHUTE || fuel.isInsideRobot()) {
      return;
    }

    for (Field2026Physics.BoxCollider box : Field2026Physics.SOLID_OBSTACLES) {
      boolean isHub = box.isHub();
      double obstacleHeight = box.height();
      double topSurfaceZ = obstacleHeight + FUEL_RADIUS;

      if (isHub) {
        double cx = 0.5 * (box.minX() + box.maxX());
        double cy = 0.5 * (box.minY() + box.maxY());
        double distFromCenter = Math.hypot(fuel.x - cx, fuel.y - cy);

        // Inside the circular funnel opening: hollow interior, never bounce off top or walls
        if (distFromCenter <= Field2026Physics.HUB_FUNNEL_RADIUS) {
          continue;
        }

        // Beveled funnel rim near top opening: gently deflect grazing shots inward into the funnel
        if (Math.abs(fuel.z - obstacleHeight) <= FUEL_RADIUS
            && distFromCenter <= Field2026Physics.HUB_FUNNEL_RADIUS + FUEL_RADIUS * 0.65) {
          double nx = (cx - fuel.x) / distFromCenter;
          double ny = (cy - fuel.y) / distFromCenter;
          fuel.vx += nx * 1.2;
          fuel.vy += ny * 1.2;
          continue;
        }
      }

      // Top-surface bounce (Trench walls, Blocker top edge, or outer flat corners of the Hub)
      if (prevZ >= topSurfaceZ && fuel.z < topSurfaceZ && box.contains(fuel.x, fuel.y, 0.0)) {
        fuel.z = topSurfaceZ;
        if (fuel.vz < 0.0) {
          fuel.vz = Math.max(0.4, -fuel.vz * FLOOR_RESTITUTION);
        }
        double cx = 0.5 * (box.minX() + box.maxX());
        double cy = 0.5 * (box.minY() + box.maxY());
        double outX = fuel.x - cx;
        double outY = fuel.y - cy;
        double outLen = Math.hypot(outX, outY);
        if (outLen > 1e-6) {
          fuel.vx += (outX / outLen) * 1.1;
          fuel.vy += (outY / outLen) * 1.1;
        }
        continue;
      }

      // Vertical side walls only exist below obstacleHeight (not up to topSurfaceZ)
      if (fuel.z >= obstacleHeight) {
        continue;
      }

      double closestX = Math.clamp(fuel.x, box.minX(), box.maxX());
      double closestY = Math.clamp(fuel.y, box.minY(), box.maxY());
      double dx = fuel.x - closestX;
      double dy = fuel.y - closestY;
      double distSq = dx * dx + dy * dy;

      if (distSq < FUEL_RADIUS * FUEL_RADIUS) {
        if (distSq > 1e-8) {
          double dist = Math.sqrt(distSq);
          double nx = dx / dist;
          double ny = dy / dist;
          fuel.x += nx * (FUEL_RADIUS - dist);
          fuel.y += ny * (FUEL_RADIUS - dist);

          double vn = fuel.vx * nx + fuel.vy * ny;
          if (vn < 0.0) {
            fuel.vx -= (1.0 + WALL_RESTITUTION) * vn * nx;
            fuel.vy -= (1.0 + WALL_RESTITUTION) * vn * ny;
          }
        } else {
          double penLeft = Math.abs(fuel.x - box.minX());
          double penRight = Math.abs(box.maxX() - fuel.x);
          double penBottom = Math.abs(fuel.y - box.minY());
          double penTop = Math.abs(box.maxY() - fuel.y);
          double minPen = Math.min(Math.min(penLeft, penRight), Math.min(penBottom, penTop));
          if (minPen == penLeft) {
            fuel.x = box.minX() - FUEL_RADIUS;
            fuel.vx = -Math.abs(fuel.vx) * WALL_RESTITUTION;
          } else if (minPen == penRight) {
            fuel.x = box.maxX() + FUEL_RADIUS;
            fuel.vx = Math.abs(fuel.vx) * WALL_RESTITUTION;
          } else if (minPen == penBottom) {
            fuel.y = box.minY() - FUEL_RADIUS;
            fuel.vy = -Math.abs(fuel.vy) * WALL_RESTITUTION;
          } else {
            fuel.y = box.maxY() + FUEL_RADIUS;
            fuel.vy = Math.abs(fuel.vy) * WALL_RESTITUTION;
          }
        }
      }
    }

    // Small triangle side blockers flanking each Neutral-Zone Hub blocker
    for (Field2026Physics.TrianglePrismCollider tri : Field2026Physics.TRIANGLE_SIDE_BLOCKERS) {
      double obstacleHeight = tri.height();
      double topSurfaceZ = obstacleHeight + FUEL_RADIUS;

      if (prevZ >= topSurfaceZ && fuel.z < topSurfaceZ && tri.contains2d(fuel.x, fuel.y)) {
        fuel.z = topSurfaceZ;
        if (fuel.vz < 0.0) {
          fuel.vz = Math.max(0.4, -fuel.vz * FLOOR_RESTITUTION);
        }
        Translation2d centroid = tri.centroid2d();
        double outX = fuel.x - centroid.getX();
        double outY = fuel.y - centroid.getY();
        double outLen = Math.hypot(outX, outY);
        if (outLen > 1e-6) {
          fuel.vx += (outX / outLen) * 1.1;
          fuel.vy += (outY / outLen) * 1.1;
        }
        continue;
      }

      if (fuel.z >= obstacleHeight) {
        continue;
      }

      if (tri.contains2d(fuel.x, fuel.y)) {
        // Ball center tunneled inside triangle: eject outward past the nearest triangle edge
        Translation2d edgePt = tri.closestPerimeterPoint2d(fuel.x, fuel.y);
        double dx = edgePt.getX() - fuel.x;
        double dy = edgePt.getY() - fuel.y;
        double dist = Math.hypot(dx, dy);
        double nx = dist > 1e-6 ? dx / dist : 0.0;
        double ny = dist > 1e-6 ? dy / dist : 1.0;
        fuel.x = edgePt.getX() + nx * FUEL_RADIUS;
        fuel.y = edgePt.getY() + ny * FUEL_RADIUS;

        double vn = fuel.vx * nx + fuel.vy * ny;
        if (vn < 0.0) {
          fuel.vx -= (1.0 + WALL_RESTITUTION) * vn * nx;
          fuel.vy -= (1.0 + WALL_RESTITUTION) * vn * ny;
        }
      } else {
        Translation2d closest = tri.closestPerimeterPoint2d(fuel.x, fuel.y);
        double dx = fuel.x - closest.getX();
        double dy = fuel.y - closest.getY();
        double dist = Math.hypot(dx, dy);
        if (dist < FUEL_RADIUS && dist > 1e-6) {
          double nx = dx / dist;
          double ny = dy / dist;
          fuel.x += nx * (FUEL_RADIUS - dist);
          fuel.y += ny * (FUEL_RADIUS - dist);

          double vn = fuel.vx * nx + fuel.vy * ny;
          if (vn < 0.0) {
            fuel.vx -= (1.0 + WALL_RESTITUTION) * vn * nx;
            fuel.vy -= (1.0 + WALL_RESTITUTION) * vn * ny;
          }
        }
      }
    }
  }

  private static void resolveFuelToFuelCollisions() {
    double minDist = 2.0 * FUEL_RADIUS;
    double minDistSq = minDist * minDist;

    for (int i = 0; i < activeFuel.size(); i++) {
      FuelPiece a = activeFuel.get(i);
      if (a.state == FuelState.SCORED_CHUTE || a.isInsideRobot()) {
        continue;
      }
      for (int j = i + 1; j < activeFuel.size(); j++) {
        FuelPiece b = activeFuel.get(j);
        if (b.state == FuelState.SCORED_CHUTE || b.isInsideRobot()) {
          continue;
        }

        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double dz = b.z - a.z;
        double distSq = dx * dx + dy * dy + dz * dz;

        if (distSq < minDistSq) {
          double dist = Math.sqrt(distSq);
          double nx = dist > 1e-6 ? dx / dist : 1.0;
          double ny = dist > 1e-6 ? dy / dist : 0.0;
          double nz = dist > 1e-6 ? dz / dist : 0.0;

          double overlap = 0.5 * (minDist - dist);
          a.x -= nx * overlap;
          a.y -= ny * overlap;
          a.z =
              Math.max(
                  Field2026Physics.getTerrainHeight(a.x, a.y) + FUEL_RADIUS, a.z - nz * overlap);
          b.x += nx * overlap;
          b.y += ny * overlap;
          b.z =
              Math.max(
                  Field2026Physics.getTerrainHeight(b.x, b.y) + FUEL_RADIUS, b.z + nz * overlap);

          double rvx = b.vx - a.vx;
          double rvy = b.vy - a.vy;
          double rvz = b.vz - a.vz;
          double velAlongNormal = rvx * nx + rvy * ny + rvz * nz;

          if (velAlongNormal < 0.0) {
            double impulse = -0.5 * (1.0 + BALL_RESTITUTION) * velAlongNormal;
            a.vx -= impulse * nx;
            a.vy -= impulse * ny;
            a.vz -= impulse * nz;
            b.vx += impulse * nx;
            b.vy += impulse * ny;
            b.vz += impulse * nz;

            if (a.state == FuelState.ON_FIELD && Math.hypot(a.vx, a.vy) > 0.08) {
              a.state = FuelState.IN_FLIGHT;
            }
            if (b.state == FuelState.ON_FIELD && Math.hypot(b.vx, b.vy) > 0.08) {
              b.state = FuelState.IN_FLIGHT;
            }
          }
        }
      }
    }
  }

  private static boolean hitsAnyFieldObstacle(double x, double y, double z) {
    for (Field2026Physics.BoxCollider box : Field2026Physics.SOLID_OBSTACLES) {
      if (z >= box.height() + FUEL_RADIUS) {
        continue;
      }
      if (box.isHub()
          && Math.hypot(x - 0.5 * (box.minX() + box.maxX()), y - 0.5 * (box.minY() + box.maxY()))
              <= Field2026Physics.HUB_FUNNEL_RADIUS) {
        continue;
      }
      if (box.contains(x, y, FUEL_RADIUS)) {
        return true;
      }
    }
    for (Field2026Physics.TrianglePrismCollider tri : Field2026Physics.TRIANGLE_SIDE_BLOCKERS) {
      if (z >= tri.height() + FUEL_RADIUS) {
        continue;
      }
      Translation2d closest = tri.closestPoint2d(x, y);
      if (Math.hypot(x - closest.getX(), y - closest.getY()) < FUEL_RADIUS) {
        return true;
      }
    }
    return false;
  }

  private static Pose3d[] computeTrajectoryPreview(Pose3d robot3d, Translation2d robotVel) {
    var poses = new ArrayList<Pose3d>();
    Pose3d start = robot3d.plus(SHOT_ORIGIN);
    double speed = computeLaunchSpeed();
    Translation3d worldVel =
        new Translation3d(
                speed * Math.cos(LAUNCH_PITCH_RAD), 0.0, speed * Math.sin(LAUNCH_PITCH_RAD))
            .rotateBy(robot3d.getRotation());

    double x = start.getX();
    double y = start.getY();
    double z = start.getZ();
    double vx = robotVel.getX() + worldVel.getX();
    double vy = robotVel.getY() + worldVel.getY();
    double vz = worldVel.getZ();
    double subDt = DT / PHYSICS_SUBSTEPS;

    for (int step = 0;
        step < 360 && z > Field2026Physics.getTerrainHeight(x, y) + FUEL_RADIUS;
        step++) {
      if (step % 10 == 0) {
        poses.add(new Pose3d(x, y, z, Rotation3d.ZERO));
      }
      double prevZ = z;
      double vMag = Math.sqrt(vx * vx + vy * vy + vz * vz);
      vx -= AIR_DRAG * vMag * vx * subDt;
      vy -= AIR_DRAG * vMag * vy * subDt;
      vz -= (GRAVITY + AIR_DRAG * vMag * vz) * subDt;
      x += vx * subDt;
      y += vy * subDt;
      z += vz * subDt;

      if (vz < 0.0
          && prevZ >= Field2026Physics.HUB_FUNNEL_HEIGHT - FUEL_RADIUS
          && z < Field2026Physics.HUB_FUNNEL_HEIGHT
          && (isInsideHubFunnel(x, y, Field2026Physics.BLUE_HUB_CENTER)
              || isInsideHubFunnel(x, y, Field2026Physics.RED_HUB_CENTER))) {
        poses.add(new Pose3d(x, y, Field2026Physics.HUB_FUNNEL_HEIGHT, Rotation3d.ZERO));
        break;
      }

      if (hitsAnyFieldObstacle(x, y, z)) {
        poses.add(new Pose3d(x, y, z, Rotation3d.ZERO));
        break;
      }
    }
    return poses.toArray(Pose3d[]::new);
  }

  private static void updateVisualization() {
    Pose3d robot3d = robotPose3dSupplier.get();
    Translation2d robotVel = robotVelocitySupplier.get();
    int storedFuelCount = getStoredFuelCount();

    // Publish all active Fuel pieces (field, airborne, intaking, hopper, and feeder) in one
    // Pose3d[]
    Pose3d[] allFuelPoses =
        activeFuel.stream().map(piece -> piece.toWorldPose3d(robot3d)).toArray(Pose3d[]::new);
    table.log("FieldFuel", allFuelPoses, Pose3d.struct);

    // Log live predicted aiming arc while flywheel is spinning or shooting
    if (intakeLauncherVoltsSupplier.getAsDouble() > 0.5 && storedFuelCount > 0) {
      table.log("TrajectoryPreview", computeTrajectoryPreview(robot3d, robotVel), Pose3d.struct);
    } else {
      table.log("TrajectoryPreview", new Pose3d[0], Pose3d.struct);
    }

    // Log match metrics
    table.log("StoredFuel", (double) storedFuelCount);
    table.log("BlueScore", (double) blueScore);
    table.log("RedScore", (double) redScore);
  }
}
