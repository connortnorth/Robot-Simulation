/*
 * Copyright 2026 FRCSoftware
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
package first.robot.simulation;

import static org.wpilib.units.Units.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import first.robot.SwerveDrivetrain;
import first.robot.SwerveModule;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.OnboardIMUSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;

public class DrivetrainSim {
  private static final double kDt = 0.02;
  private static final double kBusVoltage = 12.0;

  private final SwerveDrivetrain drivetrain;
  private final SwerveModule[] modules;
  private final SparkMaxSim[] driveMotorSims;
  private final SparkMaxSim[] steerMotorSims;
  private final SparkRelativeEncoderSim[] driveEncoderSims;
  private final SparkRelativeEncoderSim[] steerEncoderSims;
  private final double[] drivePositionsMeters;

  private final TelemetryTable table = Telemetry.getTable("Drivetrain");
  private Pose2d currentPose = new Pose2d(2.5, 2.0, Rotation2d.ZERO);
  private Pose3d currentPose3d = Pose3d.ZERO;
  private Translation2d fieldVelocity = Translation2d.ZERO;

  public DrivetrainSim(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.modules = drivetrain.getModules();

    int count = modules.length;
    this.driveMotorSims = new SparkMaxSim[count];
    this.steerMotorSims = new SparkMaxSim[count];
    this.driveEncoderSims = new SparkRelativeEncoderSim[count];
    this.steerEncoderSims = new SparkRelativeEncoderSim[count];
    this.drivePositionsMeters = new double[count];

    for (int i = 0; i < count; i++) {
      driveMotorSims[i] = new SparkMaxSim(modules[i].getDriveMotor(), DCMotor.getNEO(1));
      steerMotorSims[i] = new SparkMaxSim(modules[i].getSteerMotor(), DCMotor.getNEO(1));
      driveEncoderSims[i] = driveMotorSims[i].getRelativeEncoderSim();
      steerEncoderSims[i] = steerMotorSims[i].getRelativeEncoderSim();
    }

    setPose(new Pose2d(2.5, 2.0, Rotation2d.ZERO));
    FuelSim.robotPoseSupplier = this::getPose;
    FuelSim.robotPose3dSupplier = this::getPose3d;
    FuelSim.robotVelocitySupplier = this::getFieldVelocity;
  }

  public void periodic() {
    double rollRad = currentPose3d.getRotation().getX();
    double pitchRad = currentPose3d.getRotation().getY();
    double gradeVx = Math.sin(pitchRad) * 0.65;
    double gradeVy = -Math.sin(rollRad) * 0.65;

    double sumVx = 0.0;
    double sumVy = 0.0;
    double sumOmegaNum = 0.0;
    double sumOmegaDen = 0.0;

    for (int i = 0; i < modules.length; i++) {
      SwerveModule module = modules[i];

      // Simulate azimuth steer motor rotation
      double steerThrottle = Math.clamp(module.getSteerMotor().getThrottle(), -1.0, 1.0);
      double steerOmega = steerThrottle * SwerveModule.kMaxSteerRadPerSec;
      double newAngleRad = MathUtil.angleModulus(module.getAngleRad() + steerOmega * kDt);

      // Simulate drive wheel speed with inertia and 2026 Bump grade load
      double driveThrottle = Math.clamp(module.getDriveMotor().getThrottle(), -1.0, 1.0);
      double gradeAlongWheel = gradeVx * Math.cos(newAngleRad) + gradeVy * Math.sin(newAngleRad);
      double targetSpeedMps =
          driveThrottle * SwerveDrivetrain.kMaxSpeedMetersPerSecond + gradeAlongWheel;
      double newVelMps =
          module.getVelocityMps() + (targetSpeedMps - module.getVelocityMps()) * 0.35;

      module.updateMeasuredState(newAngleRad, newVelMps);
      drivePositionsMeters[i] += newVelMps * kDt;

      // Update REV SparkMax encoder telemetry
      double wheelRadPerSec = newVelMps / SwerveDrivetrain.kWheelRadiusMeters;
      double driveMotorRpm =
          Units.radiansPerSecondToRotationsPerMinute(
              wheelRadPerSec * SwerveDrivetrain.kDriveGearRatio);
      driveMotorSims[i].iterate(driveMotorRpm, kBusVoltage, kDt);
      driveEncoderSims[i].setPosition(
          Units.radiansToRotations(
              drivePositionsMeters[i]
                  / SwerveDrivetrain.kWheelRadiusMeters
                  * SwerveDrivetrain.kDriveGearRatio));
      driveEncoderSims[i].setVelocity(driveMotorRpm);

      double steerMotorRpm =
          Units.radiansPerSecondToRotationsPerMinute(steerOmega * SwerveDrivetrain.kSteerGearRatio);
      steerMotorSims[i].iterate(steerMotorRpm, kBusVoltage, kDt);
      steerEncoderSims[i].setPosition(
          Units.radiansToRotations(newAngleRad * SwerveDrivetrain.kSteerGearRatio));
      steerEncoderSims[i].setVelocity(steerMotorRpm);

      // Accumulate forward kinematics from module state
      double modVx = newVelMps * Math.cos(newAngleRad);
      double modVy = newVelMps * Math.sin(newAngleRad);
      Translation2d loc = module.getLocation();

      sumVx += modVx;
      sumVy += modVy;
      sumOmegaNum += (-modVx * loc.getY() + modVy * loc.getX());
      sumOmegaDen += (loc.getX() * loc.getX() + loc.getY() * loc.getY());
    }

    double robotVx = sumVx / modules.length;
    double robotVy = sumVy / modules.length;
    double robotOmega = sumOmegaDen > 1e-6 ? sumOmegaNum / sumOmegaDen : 0.0;

    // Integrate 2D robot pose using midpoint heading
    Pose2d prevPose = currentPose;
    Rotation2d midHeading = prevPose.getRotation().plus(new Rotation2d(0.5 * robotOmega * kDt));
    Translation2d fieldDelta = new Translation2d(robotVx * kDt, robotVy * kDt).rotateBy(midHeading);

    Pose2d rawPose =
        new Pose2d(
            prevPose.getTranslation().plus(fieldDelta),
            prevPose.getRotation().plus(new Rotation2d(robotOmega * kDt)));

    // Resolve 2026 field perimeter, Hub, and Trench wall collisions
    currentPose = Field2026Physics.resolveRobotCollisions(rawPose);
    drivetrain.updateHeading(currentPose.getRotation());

    // Compute true field velocity (used by FuelSim for shooting on the move)
    fieldVelocity = currentPose.getTranslation().minus(prevPose.getTranslation()).div(kDt);

    // Compute 3D Z-elevation, pitch, and roll over the Bumps
    currentPose3d = Field2026Physics.computeRobotPose3d(currentPose);

    OnboardIMUSim.setYaw(currentPose.getRotation().getRadians());

    table.log("Pose", currentPose, Pose2d.struct);
    table.log("Pose3d", currentPose3d, Pose3d.struct);
    table.log("Vx", MetersPerSecond.of(robotVx));
    table.log("Vy", MetersPerSecond.of(robotVy));
    table.log("Omega", RadiansPerSecond.of(robotOmega));
  }

  public void setPose(Pose2d pose) {
    currentPose = Field2026Physics.resolveRobotCollisions(pose);
    currentPose3d = Field2026Physics.computeRobotPose3d(currentPose);
    drivetrain.updateHeading(currentPose.getRotation());
  }

  public Pose2d getPose() {
    return currentPose;
  }

  public Pose3d getPose3d() {
    return currentPose3d;
  }

  public Translation2d getFieldVelocity() {
    return fieldVelocity;
  }
}
