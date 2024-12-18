// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

import org.assabet.aztechs157.input.layouts.Layout;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.data.LoggingSystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private Map<String,Subsystem> SystemMap = new HashMap<String, Subsystem>();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final DriveSystem drivetrain = TunerConstants.DriveTrain; // My drivetrain
  {SystemMap.put("Drive",drivetrain);}
  private final Layout inputs = Inputs.createFromChooser();
  private final LoggingSystem loggingSystem = new LoggingSystem(SystemMap);
  private final SendableChooser<Command> autoChooser;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Slew Rate Limiters to limit acceleration of joystick inputs
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(25);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(25);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1570);

  private Orchestra soundSystem = new Orchestra();

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-yLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftY(),
                        ControllerConstants.LEFT_Y_DEADBAND) * modifySpeed(MaxSpeed))) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-xLimiter.calculate(MathUtil.applyDeadband(joystick.getLeftX(),
                        ControllerConstants.LEFT_X_DEADBAND) * modifySpeed(MaxSpeed))) // Drive left with negative X (left)
            .withRotationalRate(-rotLimiter.calculate(MathUtil.applyDeadband(joystick.getRightX(),
                        ControllerConstants.RIGHT_X_DEADBAND) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-MathUtil.applyDeadband(joystick.getLeftY(),
                        ControllerConstants.LEFT_Y_DEADBAND) * modifySpeed(MaxSpeed), -MathUtil.applyDeadband(joystick.getLeftX(),
                        ControllerConstants.LEFT_X_DEADBAND) * modifySpeed(MaxSpeed)))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.start().onTrue(drivetrain.runOnce(()-> {soundSystem.play();}));
    joystick.back().onTrue(drivetrain.runOnce(()->{soundSystem.pause();}));
    joystick.y().onTrue(drivetrain.runOnce(()->{soundSystem.stop();}));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public double modifySpeed(final double speed) {
    final var modifier = 1 - inputs.axis(Inputs.precisionDrive).get();
    return speed * modifier;
}

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser("NothingAuto");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    for(int i = 0; i<4;i++){
      soundSystem.addInstrument(drivetrain.getModule(i).getDriveMotor(), 0);
      soundSystem.addInstrument(drivetrain.getModule(i).getSteerMotor(), 1);
    }
    soundSystem.loadMusic("music/e1m1.chrp");
  }

      /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return autoChooser.getSelected();
  }
  
}
