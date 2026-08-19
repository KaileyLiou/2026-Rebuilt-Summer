package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.IntakePorts;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;
    public final PIDController anglePid;
    
    public Intake() {


        // configureTalonMotor(intakeMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Coast);
        // configureTalonMotor(angleMotor, Constants.IntakeConstants.ANGLE_CURRENT_LIMIT, NeutralModeValue.Brake);

        intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS);
        angleMotor = new TalonFX(IntakePorts.INTAKE_ANGLE_MOTOR, IntakeConstants.CANBUS);
        encoder = new DutyCycleEncoder(IntakePorts.ENCODER);

        anglePid = new PIDController(IntakeConstants.PIDConstants.kP, IntakeConstants.PIDConstants.kI, IntakeConstants.PIDConstants.kD);
    }

    public double getAngle() {
        return encoder.get() * 360.0;
    }

    public void setAnglePid(double setpoint) {
        double voltage = anglePid.calculate(getAngle(), setpoint);
        angleMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    }

    private void setIntakeNeutralMode(NeutralModeValue mode) {
        var configs = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        configs.NeutralMode = mode;
        angleMotor.getConfigurator().apply(configs);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("At Correct Angle", atAngle());
        SmartDashboard.putNumber("Intake Angle", getAngle());
    }
}
