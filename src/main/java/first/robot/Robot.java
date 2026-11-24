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
import first.robot.simulation.CameraSim;
import first.robot.simulation.DrivetrainSim;
import first.robot.simulation.FuelSim;
import first.robot.simulation.SingleFlywheelSim;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Pose2d;

public class Robot extends OpModeRobot {

  public final SparkMax intakeLauncher = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
  public final SparkMax feeder = new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless);

  private final SingleFlywheelSim intakeLauncherSim =
      SingleFlywheelSim.forIntakeLauncher(intakeLauncher);
  private final SingleFlywheelSim feederSim = SingleFlywheelSim.forFeeder(feeder);

  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  private final DrivetrainSim drivetrainSim = new DrivetrainSim(drivetrain);

  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);

  private final CameraSim cameraSim = new CameraSim(drivetrainSim::getPose);

  public Robot() {
    drivetrain.setRedAlliance(true);

    SparkMaxConfig accessoryConfig = new SparkMaxConfig();
    intakeLauncher.configure(
        accessoryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder.configure(
        accessoryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Returns the simulated robot's current pose on the field. */
  public Pose2d getPose() {
    return drivetrainSim.getPose();
  }

  @Override
  public void simulationPeriodic() {
    drivetrainSim.periodic();
    cameraSim.periodic(); // Updates simulated camera values every 20ms
    intakeLauncherSim.periodic();
    feederSim.periodic();
    FuelSim.periodic();
  }
}
