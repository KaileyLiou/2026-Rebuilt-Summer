package frc.robot.subsystems;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.HopperPorts;
import frc.robot.Ports.ShooterPorts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void runHopper() {
        indexerMotor.set(ShooterConstants.INDEXER_MOTOR_SPEED);
        spindexerMotor.set(HopperConstants.MOTOR_SPEED);
    }

    public Command reverseHopperCmd() {
        return this.run(() -> reverseHopper());
    }

    public void reverseHopper() {
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
        config.CurrentLimits.SupplyCurrentLimit = currentlimit;
        config.MotorOutput.NeutralMode = mode;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        
    }

}