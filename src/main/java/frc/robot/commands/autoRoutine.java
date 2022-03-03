// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoRoutine extends SequentialCommandGroup {

  private final DriveTrain m_train;
  private final Shooter m_shooter;
  
 
  /** Creates a new autoRoutine. */
  public autoRoutine(DriveTrain train, Shooter shoot) {

    m_train = train;
    m_shooter = shoot;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new shoot(m_shooter,Constants.dampenSpeedShooter*0.1).withTimeout(2), new shoot(m_shooter,Constants.dampenSpeedShooter).withTimeout(5), new IndexBall(m_shooter).withTimeout(2)
     , new DriveDistancePID(m_train, -5) );
      
  }
}
