package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Elevator extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pid;

    private final SparkMax m_armMotor;
    private final RelativeEncoder m_armEncoder;
    private final SparkClosedLoopController m_armPID;

    // You'll need to adjust this based on your elevator's gearing
    private final double COUNTS_PER_INCH = 42.0; // Example value - measure this!
    private final double GRAVITY_COMPENSATION = 0.1; // Tune this value - usually between 0.05-0.2

    public Elevator() { // Replace SparkMax CANID
        motor = new SparkMax(11, MotorType.kBrushed);
        encoder = motor.getEncoder();

        // PID values need tuning for your specific elevator
        pid = new PIDController(0.1, 0, 0);

        m_armMotor = new SparkMax(12, MotorType.kBrushless);
        m_armEncoder = m_armMotor.getEncoder();
        m_armPID = m_armMotor.getClosedLoopController();

        m_armMotor.configure(Configs.MAXSwerveModule.armConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void moveArm(double amount) {
        SmartDashboard.putNumber("armMove", amount);
        SmartDashboard.putNumber("armEncoderPosition", m_armEncoder.getPosition());

        var pos = m_armEncoder.getPosition() + (amount / 2);
        m_armPID.setReference(pos, ControlType.kPosition);

        SmartDashboard.putNumber("armRefPos", pos);
    }

    public void resetArm() {
        m_armEncoder.setPosition(0);
    }

    // Returns elevator height in inches
    public double getHeight() {
        return encoder.getPosition() / COUNTS_PER_INCH;
    }

    public void setPosition(double targetHeight) {
        double output = pid.calculate(getHeight(), targetHeight);
        motor.set(output + GRAVITY_COMPENSATION);
    }

    public void moveupElevator() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Good place to update SmartDashboard values if needed
    }
}