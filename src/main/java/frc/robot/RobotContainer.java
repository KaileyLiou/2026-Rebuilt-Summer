// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.ShootPreloaded;
import frc.robot.autos.ShootTrench;
import frc.robot.commands.Shooting;
import frc.robot.subsystems.AutoShooter;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged // annotation to record data from the class
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveJoy =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController operJoy =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private final Drive drive;
  private final AutoShooter autoshooter;
  private final Intake intake;
  private final Hopper hopper;
  private final Shooting shooting;

  private SendableChooser<Command> autonChooser;
  // ArrayList<String> names = new ArrayList<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drive = new Drive();
    autoshooter = new AutoShooter(drive);
    intake = new Intake();
    hopper = new Hopper();
    shooting = new Shooting();

    configureBindings();
    configureDefaultCmds();
    configureAuton();

    SmartDashboard.putData(autoshooter);
  }

  private void configureDefaultCmds() { 
    drive.setDefaultCommand( // driving isn't an event, it's always happening, robot always listens to joysticks, becomes drivetrain's default behavior
      new RunCommand(
        () -> drive.driveRatio( // lambda, () -> means store the instructions, run it later bc getting values from joystick
          () -> MathUtil.applyDeadband(-driveJoy.getLeftY(), 0.1), // lefty is forward/backward movement
          () -> MathUtil.applyDeadband(-driveJoy.getLeftX(), 0,1), // leftx is left/right movement
          () -> MathUtil.applyDeadband(-driveJoy.getRightX(), 0,1)), // rightx is rotation
        drive)); // deadband to prevent drift, from -0.1 to 0.1 becomes 0, - because opposite (forward is negative, backwards is positive)
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    driveJoy.rightBumper().onTrue(
      drive.runOnce(() -> drive.zeroHeading()) // resetting direction/gyro call that forward
    );

    driveJoy.x().onTrue(
      autoshooter.resetMotorPositionCmd()
    );

    operJoy.leftTrigger()
      .onTrue(intake.setAngleUpDownCmd())
      .onFalse(intake.stopAngleUpDownCmd());

    operJoy.leftBumper()
      .whileTrue(intake.reverseIntakeMotorCmd()).onFalse(intake.stopIntakeMotorCmd());

    operJoy.rightTrigger()
      .whileTrue(shooting.prepareShot(hopper));

    operJoy.rightBumper()
      .onTrue(autoshooter.resetMotorPositionCmd());

    operJoy.povUp()
      .whileTrue(autoshooter.runAngleMotorCmd())
      .onFalse(autoshooter.stopAngleMotorCmd());

    operJoy.povDown()
      .whileTrue(autoshooter.reverseAngleMotorCmd())
      .onFalse(autoshooter.stopAngleMotorCmd());

    operJoy.povLeft()
      .whileTrue(intake.setAngleDownCmd())
      .onFalse(intake.stopAngleMotorCmd())

    operJoy.povRight()
      .whileTrue(intake.setAngleUpCmd())
      .onFalse(intake.stopAngleMotorCmd());

    operJoy.start()
      .whileTrue(hopper.reverseHopperCmd());

    operJoy.back()
      .whileTrue(autoshooter.stopShooterMotorCmd());

    operJoy.a()
      .whileTrue(autoshooter.shootHubFlushCmd())
      .onFalse(autoshooter.stopShooterMotorCmd());

    operJoy.x()
      .whileTrue(autoshooter.shootTrenchCmd())
      .onFalse(autoshooter.stopShooterMotorCmd());

    operJoy.y()
      .whileTrue(autoshooter.testShootCmd())
      .onFalse(autoshooter.stopShooterMotorCmd());

    operJoy.b()
      .whileTrue(intake.setIntakeMotorCmd())
      .onFalse(intake.stopIntakeMotorCmd());
  }

  public void configureAuton() {
    autonChooser.addOption("Shoot Preloaded", new ShootPreloaded(shooting, autoshooter, hopper));
    autonChooser.addOption("Shoot Trench", new ShootTrench(shooting, autoshooter, hopper));
    SmartDashboard.putData("Choose auto: ", autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return autonChooser.getSelected();
  }
}
