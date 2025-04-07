package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
          / ModuleConstants.kDrivingMotorReduction;

      // Calculated based off of the turning gear ratio of the SDS MK4i
      // swerve module (150/7) and conversion from rotations to radians (2pi)
      double turningFactor = (2 * Math.PI * 7.0) / 150.0;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(20)
          .inverted(false);
      turningConfig.encoder
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1.0, 0, 0)
          .outputRange(-1, 1);
      // MAKE SURE TO DISABLE .positionWrappingEnabled, AS THAT CAUSES STRANGE
      // PROBLEMS THAT CAUSED HEADACHES DURING COMPETITION IN 2025

      armConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(false);
      armConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);
      armConfig.encoder
          .positionConversionFactor(2 * Math.PI) // radians
          .velocityConversionFactor((Math.PI / 3) / 60.0); // radians per second
      armConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(2, 0, 0)
          .outputRange(-1, 1);
    }
  }
}
