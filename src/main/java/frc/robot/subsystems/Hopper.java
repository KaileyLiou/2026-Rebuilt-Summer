import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.HopperPorts;
import frc.robot.Ports.ShooterPorts;
import com.ctre.phoenix6.hardware.TalonFX;

public class Hopper extends SubsystemBase {
    private final TalonFX spindexerMotor;
    private final TalonFX indexerMotor;

    public Hopper() {
        spindexerMotor = new TalonFX(HopperPorts.HOPPER_MOTOR, HopperConstants.canbus);
        indexerMotor = new TalonFX(ShooterPorts.INDEXER_MOTOR, ShooterConstants.CANBUS);

        configureTalonMotor(spindexerMotor, HopperConstants.HOPPER_CURRENT_LIMIT, NeutralModeValue.Brake);
        configureTalonMotor(indexerMotor, ShooterConstants.CURRENT_LIMIT, NeutralModeValue.Coast);
    }

    public Command runIndexerMotorCmd() {
        return this.run(() -> indexerMotor.set(ShooterConstants.INDEXER_MOTOR_SPEED));
    }

    public Command reverseIndexerMotorCmd() {
        return this.run(() -> indexerMotor.set(-ShooterConstants.INDEXER_MOTOR_SPEED));
    } 

    public Command stopIndexerMotorCmd() {
        return this.runOnce(() -> indexerMotor.set(0));
    }

    public Command runHopperCmd() {
        return this.run(() -> runHopper());
    }

    public Command runHopper() {
        indexerMotor.set(ShooterConstants.INDEXER_MOTOR_SPEED);
        spindexerMotor.set(HopperConstants.MOTOR_SPEED);
    }

    public Command reverHopperCmd() {
        return this.run(() -> reverHopper());
    }

    public Command reverseHopper() {
        indexerMotor.set(-ShooterConstants.INDEXER_MOTOR_SPEED);
        spindexerMotor.set(-HopperConstants.MOTOR_SPEED);
    }

    public Command runSpindexer() {
        return this.run(() -> spindexerMotor.set(HopperConstants.MOTOR_SPEED));
    }

    public Command reverseSpindexer() {
        return this.run(() -> spindexerMotor.set(-HopperConstants.MOTOR_SPEED));
    }

    public Command stopSpindexer() {
        return this.runOnce(() -> spindexerMotor.set(0));
    }

    public static void configureTalonMotor(TalonFX motor, double currentlimit, NeutralModeValue mode) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.MotorOutput.NeutralModeValue = mode;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        
    }

}