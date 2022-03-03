// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class shoot extends CommandBase {
  private Shooter Shoot;
  private double power;
  /** Creates a new shoot. */

  public shoot(Shooter m_Shoot,double m_power) {

    Shoot = m_Shoot;
    power = m_power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 
    Shoot.shoot(Constants.dampenSpeedShooter);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  Shoot.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
