package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.IntakePorts;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX angleMotor;
    private final DutyCycleEncoder encoder;
    
    public Intake() {


        // configureTalonMotor(intakeMotor, Constants.IntakeConstants.INTAKE_CURRENT_LIMIT, NeutralModeValue.Coast);
        // configureTalonMotor(angleMotor, Constants.IntakeConstants.ANGLE_CURRENT_LIMIT, NeutralModeValue.Brake);

        intakeMotor = new TalonFX(IntakePorts.INTAKE_MOTOR, IntakeConstants.CANBUS);
        angleMotor = new TalonFX(IntakePorts.INTAKE_ANGLE_MOTOR, IntakeConstants.CANBUS);
        encoder = new DutyCycleEncoder(IntakePorts.ENCODER);

        
    }
}
