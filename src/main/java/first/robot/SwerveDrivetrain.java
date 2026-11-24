/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class SwerveDrivetrain {
  // Physical swerve parameters (SDS MK4i L2 equivalent)
  public static final double kMaxSpeedMetersPerSecond = 4.5;
  public static final double kMaxAngularSpeedRadPerSec = 2.0 * Math.PI;
  public static final double kWheelRadiusMeters = 0.0508; // 2-inch radius (4-inch wheel)
  public static final double kDriveGearRatio = 6.75;
  public static final double kSteerGearRatio = 150.0 / 7.0;

  private static final double kLoopDtSeconds = 0.02;
  private static final Rotation2d kRedAlliancePerspective = new Rotation2d(Math.PI);

  // Module offsets from robot center (meters), fitting inside the 0.84m x 0.76m chassis
  private static final double kModuleOffsetX = 0.32;
  private static final double kModuleOffsetY = 0.28;

  private final SwerveModule frontLeft =
      new SwerveModule(0, 6, false, false, new Translation2d(kModuleOffsetX, kModuleOffsetY));
  private final SwerveModule frontRight =
      new SwerveModule(1, 7, false, false, new Translation2d(kModuleOffsetX, -kModuleOffsetY));
  private final SwerveModule rearLeft =
      new SwerveModule(2, 8, false, false, new Translation2d(-kModuleOffsetX, kModuleOffsetY));
  private final SwerveModule rearRight =
      new SwerveModule(3, 9, false, false, new Translation2d(-kModuleOffsetX, -kModuleOffsetY));

  private final SwerveModule[] modules = {frontLeft, frontRight, rearLeft, rearRight};
  private final double[] moduleSpeeds = new double[modules.length];
  private final Rotation2d[] moduleAngles = new Rotation2d[modules.length];

  private Rotation2d alliancePerspective = kRedAlliancePerspective;
  private Rotation2d headingOffset = Rotation2d.ZERO;
  private Rotation2d rawHeading = Rotation2d.ZERO;

  /**
   * Drives the swerve drivetrain using normalized [-1, 1] inputs.
   *
   * @param xSpeed Forward/backward throttle (+ is forward away from the driver wall)
   * @param ySpeed Left/right strafe throttle (+ is left from the driver perspective)
   * @param rotSpeed Rotational throttle (+ is counter-clockwise)
   * @param fieldRelative True for field-oriented control, false for robot-oriented
   */
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    if (Math.abs(xSpeed) < 1e-4 && Math.abs(ySpeed) < 1e-4 && Math.abs(rotSpeed) < 1e-4) {
      stop();
      return;
    }

    double vx = xSpeed * kMaxSpeedMetersPerSecond;
    double vy = ySpeed * kMaxSpeedMetersPerSecond;
    double omega = rotSpeed * kMaxAngularSpeedRadPerSec;

    if (fieldRelative) {
      Rotation2d driverRelativeHeading = getHeading().minus(alliancePerspective);
      Translation2d fieldVel = new Translation2d(vx, vy);
      Translation2d robotRelative = fieldVel.rotateBy(driverRelativeHeading.unaryMinus());

      // Compensate for half-step heading rotation during the 20ms loop so translation stays
      // aligned to the field while turning simultaneously
      double maxUnscaledSpeed =
          computeMaxModuleSpeed(robotRelative.getX(), robotRelative.getY(), omega);
      double omegaScale =
          maxUnscaledSpeed > kMaxSpeedMetersPerSecond
              ? kMaxSpeedMetersPerSecond / maxUnscaledSpeed
              : 1.0;
      double effectiveOmega = omega * omegaScale;

      Translation2d compensated =
          fieldVel.rotateBy(
              driverRelativeHeading
                  .plus(new Rotation2d(0.5 * effectiveOmega * kLoopDtSeconds))
                  .unaryMinus());
      vx = compensated.getX();
      vy = compensated.getY();
    }

    double maxCalculatedSpeed = kMaxSpeedMetersPerSecond;
    for (int i = 0; i < modules.length; i++) {
      Translation2d loc = modules[i].getLocation();
      double moduleVx = vx - omega * loc.getY();
      double moduleVy = vy + omega * loc.getX();

      moduleSpeeds[i] = Math.hypot(moduleVx, moduleVy);
      moduleAngles[i] = new Rotation2d(Math.atan2(moduleVy, moduleVx));

      if (moduleSpeeds[i] > maxCalculatedSpeed) {
        maxCalculatedSpeed = moduleSpeeds[i];
      }
    }

    // Desaturate wheel speeds proportionally if combined translation + rotation exceeds max speed
    double scale = kMaxSpeedMetersPerSecond / maxCalculatedSpeed;
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(
          moduleSpeeds[i] * scale, moduleAngles[i], kMaxSpeedMetersPerSecond);
    }
  }

  private double computeMaxModuleSpeed(double vx, double vy, double omega) {
    double maxSpeed = 0.0;
    for (SwerveModule module : modules) {
      Translation2d loc = module.getLocation();
      double moduleVx = vx - omega * loc.getY();
      double moduleVy = vy + omega * loc.getX();
      double speed = Math.hypot(moduleVx, moduleVy);
      if (speed > maxSpeed) {
        maxSpeed = speed;
      }
    }
    return maxSpeed;
  }

  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /** Sets whether field-oriented driving uses the Red Alliance (-X) driver perspective. */
  public void setRedAlliance(boolean isRedAlliance) {
    this.alliancePerspective = isRedAlliance ? kRedAlliancePerspective : Rotation2d.ZERO;
  }

  public void updateHeading(Rotation2d currentFieldHeading) {
    this.rawHeading = currentFieldHeading;
  }

  public Rotation2d getHeading() {
    return rawHeading.minus(headingOffset);
  }

  /** Resets field-oriented forward away from the active alliance's driver wall. */
  public void zeroHeading() {
    headingOffset = rawHeading.minus(alliancePerspective);
  }

  public SwerveModule[] getModules() {
    return modules;
  }
}
