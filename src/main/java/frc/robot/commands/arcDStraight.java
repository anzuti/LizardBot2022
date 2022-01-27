// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.RobotContainer;
//import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj2.command.CommandBase;

public class arcDStraight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain dtrain;
  private final Joystick gamePad;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public arcDStraight(DriveTrain m_drivetrain, Joystick joy) {
   
    dtrain = m_drivetrain;
    gamePad = joy;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    dtrain.driveStraight(gamePad);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

