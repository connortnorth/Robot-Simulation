/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

public class SwerveModule {
  private static final double kLoopDtSeconds = 0.02;
  public static final double kMaxSteerRadPerSec = 12.0;

  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final Translation2d moduleLocation;

  private final PIDController steerPID = new PIDController(4.0, 0.0, 0.0);

  private double currentAngleRad = 0.0;
  private double currentVelocityMps = 0.0;
  private double prevTargetRad = 0.0;
  private boolean hasPrevTarget = false;

  public SwerveModule(
      int driveCanId,
      int steerCanId,
      boolean driveInverted,
      boolean steerInverted,
      Translation2d moduleLocation) {
    this.driveMotor = new SparkMax(CANPort.CAN_S0, driveCanId, MotorType.kBrushless);
    this.steerMotor = new SparkMax(CANPort.CAN_S0, steerCanId, MotorType.kBrushless);
    this.moduleLocation = moduleLocation;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveInverted);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig steerConfig = new SparkMaxConfig();
    steerConfig.inverted(steerInverted);
    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** Commands the module to a target speed (m/s) and robot-relative wheel angle. */
  public void setDesiredState(double desiredSpeedMps, Rotation2d desiredAngle, double maxSpeedMps) {
    // Hold wheel angle when commanded speed is near zero to prevent snapping back to 0 deg
    if (Math.abs(desiredSpeedMps) < 0.01) {
      stop();
      return;
    }

    double targetRad = desiredAngle.getRadians();
    double deltaRad = MathUtil.angleModulus(targetRad - currentAngleRad);

    // Optimize azimuth rotation so the wheel never turns more than 90 degrees
    if (Math.abs(deltaRad) > Math.PI / 2.0) {
      desiredSpeedMps = -desiredSpeedMps;
      targetRad = MathUtil.angleModulus(targetRad + Math.PI);
      deltaRad = MathUtil.angleModulus(targetRad - currentAngleRad);
    }

    // Track continuous target azimuth velocity so wheels stay aligned while the robot rotates
    double steerFeedforward = 0.0;
    if (hasPrevTarget) {
      double deltaTarget = MathUtil.angleModulus(targetRad - prevTargetRad);
      if (Math.abs(deltaTarget) < Math.PI / 4.0) {
        double targetOmega = deltaTarget / kLoopDtSeconds;
        steerFeedforward = targetOmega / kMaxSteerRadPerSec;
      }
    }
    prevTargetRad = targetRad;
    hasPrevTarget = true;

    // Scale drive speed by cos(error) to avoid scrubbing while the wheel is turning
    double scaledSpeedMps = desiredSpeedMps * Math.max(0.0, Math.cos(deltaRad));

    double driveThrottle = Math.clamp(scaledSpeedMps / maxSpeedMps, -1.0, 1.0);
    double steerThrottle =
        Math.clamp(steerPID.calculate(currentAngleRad, targetRad) + steerFeedforward, -1.0, 1.0);

    driveMotor.setThrottle(driveThrottle);
    steerMotor.setThrottle(steerThrottle);
  }

  public void stop() {
    driveMotor.setThrottle(0.0);
    steerMotor.setThrottle(0.0);
    steerPID.reset();
    hasPrevTarget = false;
  }

  /** Updates the module's measured angle (rad) and wheel speed (m/s) from simulation/encoders. */
  public void updateMeasuredState(double angleRad, double velocityMps) {
    this.currentAngleRad = MathUtil.angleModulus(angleRad);
    this.currentVelocityMps = velocityMps;
  }

  public SparkMax getDriveMotor() {
    return driveMotor;
  }

  public SparkMax getSteerMotor() {
    return steerMotor;
  }

  public Translation2d getLocation() {
    return moduleLocation;
  }

  public double getAngleRad() {
    return currentAngleRad;
  }

  public double getVelocityMps() {
    return currentVelocityMps;
  }
}
