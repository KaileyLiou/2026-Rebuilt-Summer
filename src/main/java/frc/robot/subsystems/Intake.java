package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.IntakePorts;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;
    private final PIDController anglePid;
    private final double errorMargin; 
    
    public Intake() {
        intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS);
        angleMotor = new TalonFX(IntakePorts.INTAKE_ANGLE_MOTOR, IntakeConstants.CANBUS);
        encoder = new DutyCycleEncoder(IntakePorts.ENCODER);

        configureTalonMotor(intakeMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Coast);
        configureTalonMotor(angleMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Brake);

        errorMargin = 45;

        anglePid = new PIDController(IntakeConstants.PIDConstants.kP, IntakeConstants.PIDConstants.kI, IntakeConstants.PIDConstants.kD);
        anglePid.setTolerance(0.05); // consider the mechanism at the setpoint when the error is within 0.05 of the setpoint
    }

    public double getAngle() {
        return encoder.get() * 360.0; // convert from rotations to deg
    }

    public void setAnglePid(double setpoint) {
        double voltage = anglePid.calculate(getAngle(), setpoint);
        angleMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    public boolean atAngle() {
        return anglePid.atSetpoint();
    }

    private void setIntakeNeutralMode(NeutralModeValue mode) {
        var configs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        configs.NeutralMode = mode;
        angleMotor.getConfigurator().apply(configs);
    }

    public static void configureTalonMotor(TalonFX motor, double currentLimit, NeutralModeValue mode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.MotorOutput.NeutralMode = mode;
        
        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Correct Angle", atAngle());
        SmartDashboard.putNumber("Intake Angle", getAngle());
    }
}
