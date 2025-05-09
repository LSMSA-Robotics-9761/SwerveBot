// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;

public class SwerveModule {
  // Determines which of the four swerve modules on the robot the instance is. Can
  // be "fr", "fl", "br", "bl"
  private final String position;

  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  // Relative encoders for each of the driving and turning motors
  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  // Absolute encoders for the turning motor. Used to straighten the wheels after
  // power cycles
  private final CANcoder m_turningAbsoluteEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Angular offset of turning motor determined by absolute encoder
  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to a swerve
   * module build with NEOs, SPARKS MAX, and a CANcoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int turningAbsoluteCANId, double chassisAngularOffset,
      String position) {
    this.position = position;

    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();

    m_turningAbsoluteEncoder = new CANcoder(turningAbsoluteCANId);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(
        Units.rotationsToRadians(m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
            - (Radians.of(m_chassisAngularOffset).in(Radians)));

    m_drivingEncoder.setPosition(0);

    m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getPosition().getValue().in(Radians) - chassisAngularOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    String keyPrefix = "";
    switch (position) {
      case "fr":
        keyPrefix = "frontRight";
        break;
      case "fl":
        keyPrefix = "frontLeft";
        break;
      case "br":
        keyPrefix = "backRight";
        break;
      case "bl":
        keyPrefix = "backLeft";
        break;
      default:
        break;
    }
    SmartDashboard.putNumber((keyPrefix + " absEncoderRot"),
        m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber((keyPrefix + " absEncoderRad"),
        m_turningAbsoluteEncoder.getAbsolutePosition().getValue().in(Radians));
    SmartDashboard.putNumber((keyPrefix + " absEncoderOffsettedRot"),
        m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
            - (Radians.of(m_chassisAngularOffset).in(Rotations)));
    SmartDashboard.putNumber((keyPrefix + " absEncoderOffsettedRad"),
        m_turningAbsoluteEncoder.getAbsolutePosition().getValue().in(Radians)
            - (Radians.of(m_chassisAngularOffset).in(Radians)));

    SmartDashboard.putNumber((keyPrefix + " encoderRad"),
        m_turningEncoder.getPosition());

    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
            - (Radians.of(m_chassisAngularOffset).in(Rotations))));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()
            - (Radians.of(m_chassisAngularOffset).in(Rotations))));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    SmartDashboard.putString(position + " cDS", correctedDesiredState.toString());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    // correctedDesiredState
    // .optimize(new Rotation2d(m_turningEncoder.getPosition() * 2 * Math.PI));

    SmartDashboard.putString(position + " cDS", correctedDesiredState.toString());

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(-correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
