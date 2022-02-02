// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//// IMPORT SUBSYSTEMS ///////////////////////
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//// IMPORT COMMANDS //////////////////////////

import frc.robot.commands.*;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_intake = new Intake();
  private final Shooter m_shoot = new Shooter();
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Joystick m_joystick = new Joystick(Constants.GAME_PAD);

  private final Command m_autoCommand = new autoRoutine(m_drivetrain);
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  //METODO CONSTRUCTOR PARA ROBOTCONTAINER
  public RobotContainer() {

    m_drivetrain.calibrateGyro();   
    // Configure the button bindings
    configureButtonBindings();

    
    //set Default Command
   m_drivetrain.setDefaultCommand(new arcDrive(m_drivetrain, m_joystick));

    
  


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //final JoystickButton buttonA = new JoystickButton(m_joystick, Constants.ButtonA);
    //buttonA.whileHeld(new arcDStraight(m_drivetrain, m_joystick));  
    
    //final JoystickButton buttonX = new JoystickButton(m_joystick, Constants.ButtonX);
    //buttonX.whenPressed(new DriveDistancePID(m_drivetrain,5));  


    //final JoystickButton buttonB = new JoystickButton(m_joystick, Constants.ButtonB);
    //buttonB.whenPressed(new rotateTo(m_drivetrain,90));  

    //final JoystickButton buttonY = new JoystickButton(m_joystick, Constants.ButtonY);
    //buttonY.whenPressed(new DriveDistancePID(m_drivetrain,-5)); 
    

    final JoystickButton buttonSquare = new JoystickButton(m_joystick, Constants.ButtonSquare);
    buttonSquare.whileHeld(new collectBall(m_intake));
    
    final JoystickButton buttonCircle = new JoystickButton(m_joystick, Constants.ButtonCircle);
    buttonCircle.whileHeld(new ejectBall(m_intake));

    final JoystickButton buttonTriangle = new JoystickButton(m_joystick, Constants.ButtonTriangle);
    buttonTriangle.whileHeld(new IndexBall(m_shoot));

    final JoystickButton buttonX = new JoystickButton(m_joystick, Constants.ButtonX);
    buttonX.whileHeld(new indexBallOut(m_shoot));
    
    final JoystickButton buttonR1 = new JoystickButton(m_joystick, Constants.ButtonR1);
    buttonR1.whileHeld ( new shoot(m_shoot));

  
  
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
    
  }
}
