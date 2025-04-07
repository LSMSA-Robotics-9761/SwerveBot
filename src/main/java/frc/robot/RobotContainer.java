// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.AutoDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_elevator = new Elevator();

  private boolean fieldCentered = false;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The elevator controller's controller
  CommandXboxController m_elevatorCommandController = new CommandXboxController(OIConstants.kElevatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                getRobotOrientationMode()),
            m_robotDrive));
    m_elevator.setDefaultCommand(new RunCommand(
        () -> {
          // Move arm of elevator with x-axis of right joystick using elevtor controller
          m_elevator.moveArm(
              MathUtil.applyDeadband(m_elevatorCommandController.getRightX(), OIConstants.kDriveDeadband));
        },
        m_elevator));
  }

  /**
   * Allows driver to swap between field-centered and robot-centered mode.
   */
  private boolean getRobotOrientationMode() {
    if (m_driverController.getAButton()) {
      fieldCentered = true;
      System.out.println("Set to FIELD centered");
    }

    else if (m_driverController.getBButton()) {
      fieldCentered = false;
      System.out.println("Set to ROBOT centered");
    }
    SmartDashboard.putBoolean("Field Centered", fieldCentered);
    return fieldCentered;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@linkedu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value).whileTrue(new RunCommand(
        () -> {
          m_robotDrive.zeroHeading();
          System.out.println("zeroed field heading");
        },
        m_robotDrive));

    m_elevatorCommandController.rightStick()
        .whileTrue(new RunCommand(() -> {
          m_elevator.resetArm();
          System.out.println("arm RESET");
        }, m_elevator));

    m_elevatorCommandController.x()
        .whileTrue(new RunCommand(() -> {
          m_elevator.setPosition(-3.0);
          System.out.println("elevator DOWN");
        }, m_elevator));

    m_elevatorCommandController.a()
        .whileTrue(new RunCommand(() -> {
          m_elevator.setPosition(0.0);
          System.out.println("elevator STOP");
        }, m_elevator));

    m_elevatorCommandController.b()
        .whileTrue(new RunCommand(() -> {
          m_elevator.setPosition(10.0);
          System.out.println("elevator UP FAST");
        }, m_elevator));

    m_elevatorCommandController.y()
        .whileTrue(new RunCommand(() -> {
          m_elevator.setPosition(7.0);
          System.out.println("elevator UP");
        }, m_elevator));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // config);

    // var thetaController = new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new
    // SwerveControllerCommand(
    // exampleTrajectory,
    // m_robotDrive::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_robotDrive::setModuleStates,
    // m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false));
    return new AutoDrive(m_robotDrive).withTimeout(1);
  }
}
