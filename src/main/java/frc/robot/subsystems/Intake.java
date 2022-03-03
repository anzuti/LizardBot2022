// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import libraries


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//UNUSED LIBRARIES 

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.math.controller.PIDController;
//import java.util.ResourceBundle.Control;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import edu.wpi.first.wpilibj.SpeedController;

//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.Encoder;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import edu.wpi.first.wpilibj.drive.*;
//import edu.wpi.first.wpilibj.controller.*;


// THIS IS A CHANGE I JUST MADE

import frc.robot.Constants;

public class Intake extends SubsystemBase {


//VARIABLE PARA AJUSTAR LA VELOCIAD MÁXIMA DE LOS MOTORES.
private double dampenSpeedIntake = Constants.dampenSpeedIntake;


//MOTOR GROUPS

private CANSparkMax intake ;
private CANSparkMax trigger ;




  /** Creates a new DriveTrain. *////////////////////////////////////////////////////////////////////
  public Intake()
  {
    super(); //CONSTRUYE EL OBJETO DEL PADRE.
    
    //Set motors
   // trigger = new CANSparkMax(Constants.triggerID, MotorType.kBrushed);
    intake = new CANSparkMax(Constants.intakeID, MotorType.kBrushed);

    
  }

  public void releaseIntake()
  {
    

    //trigger.set(1);
   


  }

  //HELPER METHODS FOR THE DRIVETRAIN
  public void collect() //metodo para el manejo con control libre.
  {
    intake.set(Constants.dampenSpeedIntake);
  }

  public void eject ()
  {
    intake.set(-Constants.dampenSpeedIntake);
  }

  public void stopIntake(){

    intake.set(0);
 
  }

////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {

    



    // This method will be called once per scheduler run
  }
}
