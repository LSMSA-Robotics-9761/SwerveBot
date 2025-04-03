package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pid;

    private final double COUNTS_PER_INCH = 42.0; // Example value - measure this!
    private final double GRAVITY_COMPENSATION = 0.1; // Tune this value - usually between 0.05-0.2

    public Elevator() { // Replace SparkMax CANID
        motor = new SparkMax(0, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // PID values need tuning for your specific elevator
        pid = new PIDController(0.1, 0, 0);
    }

    // Returns elevator height in inches
    public double getHeight() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public void setPosition(double targetHeight) {
        double pidOutput = pid.calculate(getHeight(), targetHeight);

        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        double motorOutput = pidOutput + GRAVITY_COMPENSATION;

        // Clamp the output to valid range
        motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

        motor.set(motorOutput);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
    }
}