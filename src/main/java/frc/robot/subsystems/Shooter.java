// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



//import libraries


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


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

public class Shooter extends SubsystemBase {

//DIGITAL INPUTS FROM THE IR SENSORS
DigitalInput irSensorFront = new DigitalInput(Constants.IRSensorFront);
DigitalInput irSensorBack = new DigitalInput(Constants.IRSensorBack);

//VARIABLE PARA AJUSTAR LA VELOCIAD MÁXIMA DE LOS MOTORES.
private double dampenSpeedShooter = Constants.dampenSpeedShooter;
private double dampenSpeedIndexer = Constants.dampenSpeedindexer;

//MOTORs

private CANSparkMax shooter ;
private CANSparkMax indexer ;





  // Create a new Shooter object
  public Shooter()
  {
    super(); //CONSTRUYE EL OBJETO DEL PADRE.
    
    //Set motors
    shooter = new CANSparkMax(Constants.shooterID, MotorType.kBrushed);
    indexer = new CANSparkMax(Constants.indexID, MotorType.kBrushed);

    
  }

  public void shootHigh()
  {
    
    shooter.set(dampenSpeedShooter);
  

  }


  public void shootLow() 
  {
    shooter.set(dampenSpeedShooter*0.5);
  }

  public void ballIn ()
  {
    indexer.set(dampenSpeedIndexer);
  }

  public void ballOut(){

    indexer.set(-dampenSpeedIndexer);
  }

  public void stopShooter(){

shooter.stopMotor();

  }

  public void stopIndexer(){


indexer.stopMotor();

  }

////////////////////////////////////////////////////////////////////////////////////////////////////
  // SENSOR GETTER METHODS

  public boolean getSensorFrontState(){

    return irSensorFront.get();

  }

  public boolean getSensorBackState(){

    return irSensorBack.get();

  }



////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {

    



    // This method will be called once per scheduler run
  }
}
