// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Dictionary;
import java.util.Map;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.LoggingConstants;

public class LoggingSystem extends SubsystemBase {
  
  private Map<String,Subsystem> subsystemArray;
  private boolean loggingState = LoggingConstants.DEFAULT_LOGGING_STATE;
  private GenericEntry loggingChooser;
  

  /** Creates a new LoggingSystem. */
  public LoggingSystem(Map<String,Subsystem> subsystemArray) {
    loggingChooser = Shuffleboard.getTab("Logging").add("Enable Logging", false).getEntry();
    SmartDashboard.putBoolean("Logging Chooser", false);
    this.subsystemArray = subsystemArray;
  }

  public boolean getLoggingState() {
    return loggingState;
  }

  private boolean getLoggingFlag() {
    // An example command will be run in autonomous
    return loggingChooser.getBoolean(false); // TODO: Actually use this value
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getLoggingFlag() != loggingState) {
      loggingState = getLoggingFlag();

      if (loggingState) {
        SignalLogger.start();
        // Log the odometry pose as a double array
      } else {
        SignalLogger.stop();
      }
    } 
    if (loggingState){
      var pose = ((CommandSwerveDrivetrain)subsystemArray.get(new String("Drive"))).getState().Pose;
      var status = SignalLogger.writeDoubleArray("odometry", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }
  }
}
