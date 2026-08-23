package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class AutoShooter extends SubsystemBase {
    private final TalonFX shooterMotor; // motor for starting the rollers
    private final TalonFX angleMotor; // motor for adjusting shooter to desired angle
    private final DutyCycleEncoder encoder;
    private final PIDController shooterPID;
    private final SimpleMotorFeedforward shooterFF;


    public static void configureTalonMotor(TalonFX motor, double currentLimit, NeutralModeValue mode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.MotorOutput.NeutralMode = mode;

        motor.getConfigurator().apply(config);
    }
}
