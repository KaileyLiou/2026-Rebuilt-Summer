package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

        // NeutralModeValue represents what the motor should do when you're not commanding it to move
        configureTalonMotor(intakeMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Coast);
        configureTalonMotor(angleMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Brake);

        errorMargin = 45;

        anglePid = new PIDController(IntakeConstants.PIDConstants.kP, IntakeConstants.PIDConstants.kI, IntakeConstants.PIDConstants.kD);
        anglePid.setTolerance(0.05); // consider the mechanism at the setpoint when the error is within 0.05 of the setpoint
    }

    public Intake(TalonFX iM, TalonFX aM, DutyCycleEncoder e, PIDController p, double eM) { // for unit testing
        intakeMotor = iM;
        angleMotor = aM;
        encoder = e;
        
        anglePid = p;
        errorMargin = eM;
    }

    public Command setAngleUpDownCmd() {
        return this.runOnce(() -> {
            double middle = (IntakeConstants.ANGLE_DOWN + IntakeConstants.ANGLE_UP)/2.0;
            if(getAngle() < middle) {
                setAnglePid(IntakeConstants.ANGLE_DOWN);
            } else {
                setAnglePid(IntakeConstants.ANGLE_UP);
            }
        });
    }

    public Command setAnglePidCmd(double setpoint) {
        return this.run(() -> setAnglePid(setpoint));
    }

    public Command setAngleUpCmd() {
        return this.run(() -> angleMotor.set(-IntakeConstants.PIVOT_SPEED));
    }

    public Command setAngleDownCmd() {
        return this.run(() -> angleMotor.set(IntakeConstants.PIVOT_SPEED));
    }

    public Command setIntakeMotorCmd() {
        return this.run(() -> intakeMotor.set(-IntakeConstants.INTAKE_MOTOR_SPEED)) // negative bc 
            .beforeStarting(() -> setIntakeNeutralMode(NeutralModeValue.Brake))
            .finallyDo((interrupted) -> { // interrupted tells you whether the command ended normally or was interrupted
                angleMotor.set(0);
                setIntakeNeutralMode(NeutralModeValue.Coast);
            });
    }

    public Command reverseIntakeMotorCmd() {
        return this.run(() -> intakeMotor.set(IntakeConstants.INTAKE_MOTOR_SPEED))
            .beforeStarting(() -> setIntakeNeutralMode(NeutralModeValue.Brake))
            .finallyDo((interrupted) -> {
                angleMotor.set(0);
                setIntakeNeutralMode(NeutralModeValue.Coast);
            });
    }

    public Command stopIntakeMotorCmd() {
        return this.runOnce(() -> intakeMotor.set(0));
    }

    public Command stopAngleMotorCmd() {
        return this.runOnce(() -> angleMotor.set(0));
    }

    public double getAngle() {
        return encoder.get() * 360.0; // convert from rotations to deg
    }

    public void setAnglePid(double setpoint) {
        double voltage = anglePid.calculate(getAngle(), setpoint);
        angleMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0)); // clamp means don't let a value go outside a certain range of the battery voltage
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
