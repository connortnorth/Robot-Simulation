/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.opmode;

import first.robot.Robot;
import first.robot.simulation.Field2026Physics;
import java.util.Set;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class MyTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final XboxController xboxController = new XboxController(0);

  // Cached NetworkTable entries published by CameraSim (or a real Limelight)
  private final NetworkTableEntry rawFiducialsEntry;
  private final NetworkTableEntry tvEntry;
  private final NetworkTableEntry tidEntry;

  // Red Alliance Hub AprilTags only (18..27) — ignores Blue Hub tags (2..11)
  private static final Set<Integer> RED_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
  private static final Rotation2d RED_ALLIANCE_PERSPECTIVE = new Rotation2d(Math.PI);

  // Driver sensitivity & deadband settings
  private static final double TRANSLATION_DEADBAND = 0.04;
  private static final double TURN_DEADBAND = 0.03;
  private static final double TURN_SENSITIVITY = 0.90;

  // Calibrated shooting distance (meters) from robot center to Red Hub funnel center at 0.90
  // throttle
  private static final double IDEAL_HUB_CENTER_DISTANCE_METERS = 2.00;
  private static final double MAX_ALIGN_SPEED = 0.85;
  private static final double ORBIT_STRAFE_SCALE = 0.55;

  // Launcher & feeder throttle parameters (NEO free speed ~5676 RPM)
  private static final double NEO_FREE_SPEED_RPM = 5676.0;
  private static final double BASE_LAUNCH_THROTTLE = 0.90;
  private static final double MIN_LAUNCH_THROTTLE = 0.86;
  private static final double MAX_LAUNCH_THROTTLE = 1.00;
  private static final double MIN_VALID_LAUNCH_RPM = 4840.0;
  private static final double FEED_THROTTLE = 1.00;
  private static final double INTAKE_THROTTLE = 0.85;
  private static final double INTAKE_FEEDER_THROTTLE = -1.0;
  private static final double OUTTAKE_THROTTLE = -0.80;
  private static final double OUTTAKE_FEEDER_THROTTLE = 0.80;

  // PID constants for turning toward the Red Hub center
  private static final double kTurnP = 0.038;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0012;
  private final PIDController turnPID = new PIDController(kTurnP, kTurnI, kTurnD);

  // PID constants for holding the ideal shooting range
  private static final double kDriveP = 1.15;
  private static final double kDriveI = 0.0;
  private static final double kDriveD = 0.08;
  private final PIDController drivePID = new PIDController(kDriveP, kDriveI, kDriveD);

  // Start in Red-alliance field-relative mode (toggled with Y; zeroed with X)
  private boolean fieldRelative = true;
  private boolean prevYButton = false;
  private boolean prevXButton = false;

  public MyTeleop(Robot robot) {
    this.robot = robot;
    robot.drivetrain.setRedAlliance(true);

    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
    this.rawFiducialsEntry = cameraTable.getEntry("rawfiducials");
    this.tvEntry = cameraTable.getEntry("tv");
    this.tidEntry = cameraTable.getEntry("tid");

    turnPID.enableContinuousInput(-180.0, 180.0);
    turnPID.setTolerance(1.5);
    drivePID.setTolerance(0.04);
  }

  @Override
  public void periodic() {
    // Toggle field-relative drive mode on Y button press
    boolean yButton = xboxController.getYButton();
    if (yButton && !prevYButton) {
      fieldRelative = !fieldRelative;
    }
    prevYButton = yButton;

    // Zero Red-alliance field-relative forward heading on X button press
    boolean xButton = xboxController.getXButton();
    if (xButton && !prevXButton) {
      robot.drivetrain.zeroHeading();
    }
    prevXButton = xButton;

    // Radial deadband for smooth diagonal translation (+X forward from Red wall, +Y left)
    double rawX = -xboxController.getLeftY();
    double rawY = -xboxController.getLeftX();
    double stickMag = Math.hypot(rawX, rawY);
    double scaledMag = MathUtil.applyDeadband(Math.min(1.0, stickMag), TRANSLATION_DEADBAND);
    double xSpeed = stickMag > 1e-6 ? (rawX / stickMag) * scaledMag : 0.0;
    double ySpeed = stickMag > 1e-6 ? (rawY / stickMag) * scaledMag : 0.0;

    // Direct linear turning response (-RightX = counter-clockwise)
    double rawTurn = MathUtil.applyDeadband(-xboxController.getRightX(), TURN_DEADBAND);
    double rotationSpeed = rawTurn * TURN_SENSITIVITY;

    Pose2d robotPose = robot.getPose();
    boolean isAutoAligning = false;

    // Hold A to lock onto the Red Hub while allowing tangential orbit strafing around blockers
    if (xboxController.getAButton() && isRedHubVisible()) {
      Translation2d hubCenter = Field2026Physics.RED_HUB_CENTER;
      double dx = hubCenter.getX() - robotPose.getX();
      double dy = hubCenter.getY() - robotPose.getY();
      double distanceToCenter = Math.hypot(dx, dy);

      if (distanceToCenter > 1e-4) {
        isAutoAligning = true;

        Rotation2d angleToCenter = new Rotation2d(Math.atan2(dy, dx));
        double angleErrorDeg =
            MathUtil.inputModulus(
                angleToCenter.minus(robotPose.getRotation()).getDegrees(), -180.0, 180.0);

        double turnOutput = -turnPID.calculate(angleErrorDeg, 0.0);
        rotationSpeed =
            turnPID.atSetpoint() ? 0.0 : Math.clamp(turnOutput, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);

        double driveOutput =
            -drivePID.calculate(distanceToCenter, IDEAL_HUB_CENTER_DISTANCE_METERS);
        double radialSpeed =
            drivePID.atSetpoint()
                ? 0.0
                : Math.clamp(driveOutput, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);

        // Project driver stick into field space to allow orbiting around the Red Hub Blocker
        Translation2d driverFieldInput =
            fieldRelative
                ? new Translation2d(xSpeed, ySpeed).rotateBy(RED_ALLIANCE_PERSPECTIVE)
                : new Translation2d(xSpeed, ySpeed).rotateBy(robotPose.getRotation());

        double ux = dx / distanceToCenter;
        double uy = dy / distanceToCenter;
        double tangentX = -uy;
        double tangentY = ux;
        double tangentialSpeed =
            (driverFieldInput.getX() * tangentX + driverFieldInput.getY() * tangentY)
                * ORBIT_STRAFE_SCALE;

        Translation2d fieldVel =
            new Translation2d(
                ux * radialSpeed + tangentX * tangentialSpeed,
                uy * radialSpeed + tangentY * tangentialSpeed);
        Translation2d robotRelVel = fieldVel.rotateBy(robotPose.getRotation().unaryMinus());

        xSpeed = Math.clamp(robotRelVel.getX(), -1.0, 1.0);
        ySpeed = Math.clamp(robotRelVel.getY(), -1.0, 1.0);
      }
    } else {
      turnPID.reset();
      drivePID.reset();
    }

    robot.drivetrain.drive(xSpeed, ySpeed, rotationSpeed, fieldRelative && !isAutoAligning);

    // Intake / Launcher / Outtake controls
    if (xboxController.getRightBumperButton()) {
      double distanceToRedHub =
          robotPose.getTranslation().getDistance(Field2026Physics.RED_HUB_CENTER);
      double launchThrottle = computeRangeCompensatedThrottle(distanceToRedHub);
      double requiredRpm =
          Math.max(MIN_VALID_LAUNCH_RPM, launchThrottle * NEO_FREE_SPEED_RPM * 0.96);

      robot.intakeLauncher.setThrottle(launchThrottle);
      Double launcherRpm = robot.intakeLauncher.getEncoder().getVelocity().get();
      if (launcherRpm != null && launcherRpm >= requiredRpm) {
        robot.feeder.setThrottle(FEED_THROTTLE);
      } else {
        robot.feeder.setThrottle(0.0);
      }
    } else if (xboxController.getLeftBumperButton()) {
      robot.intakeLauncher.setThrottle(INTAKE_THROTTLE);
      robot.feeder.setThrottle(INTAKE_FEEDER_THROTTLE);
    } else if (xboxController.getBButton()) {
      robot.intakeLauncher.setThrottle(OUTTAKE_THROTTLE);
      robot.feeder.setThrottle(OUTTAKE_FEEDER_THROTTLE);
    } else {
      robot.intakeLauncher.setThrottle(0.0);
      robot.feeder.setThrottle(0.0);
    }
  }

  /** Scales launcher throttle smoothly to match Red Hub distance across [1.82m, 2.36m]. */
  private double computeRangeCompensatedThrottle(double distanceMeters) {
    double rawThrottle =
        BASE_LAUNCH_THROTTLE + (distanceMeters - IDEAL_HUB_CENTER_DISTANCE_METERS) / 3.70;
    return Math.clamp(rawThrottle, MIN_LAUNCH_THROTTLE, MAX_LAUNCH_THROTTLE);
  }

  /** Returns true only when at least one Red Alliance Hub AprilTag (18..27) is visible. */
  private boolean isRedHubVisible() {
    double[] fiducials = rawFiducialsEntry.getDoubleArray(new double[0]);
    if (fiducials.length >= 7) {
      for (int i = 0; i + 6 < fiducials.length; i += 7) {
        int tagId = (int) Math.round(fiducials[i]);
        if (RED_HUB_TAG_IDS.contains(tagId)) {
          return true;
        }
      }
      return false;
    }

    if (tvEntry.getDouble(0.0) == 1.0) {
      int tid = (int) Math.round(tidEntry.getDouble(-1));
      return RED_HUB_TAG_IDS.contains(tid);
    }

    return false;
  }
}
