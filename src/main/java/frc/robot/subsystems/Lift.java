// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



//import libraries


import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



import frc.robot.Constants;    //import the constants from the class

public class Lift extends SubsystemBase {

//DIGITAL INPUTS FROM LIMITSWITCH SENSOR

DigitalInput liftLimitSwitch = new DigitalInput(Constants.liftLimitSwitch);
private boolean limitState=false;


//VARIABLE PARA AJUSTAR LA VELOCIAD MÁXIMA DE LOS MOTORES DEL LIFT
private double dampenSpeedLift = Constants.dampenSpeedLift;
private double dampenSpeedWinch = Constants.dampenSpeedWinch;

//MOTORs

private CANSparkMax liftMotor ;
private WPI_TalonSRX winchMotor ;







  /** Creates a new lift  *////////////////////////////////////////////////////////////////////
  public Lift()
  {
    super(); //CONSTRUYE EL OBJETO DEL PADRE.
    
    //Set motors
    liftMotor = new CANSparkMax(Constants.liftID, MotorType.kBrushed);
    winchMotor = new WPI_TalonSRX(Constants.liftWinch);

    
  }

// METHOD TO EXTEND OR RETRACT ARM FOR LIFT
  public void extendArm(double speed){


    liftMotor.set(speed);

  }

//LIFT ROBOT WITH WINCH METHOD

  public void liftRobot(double speed){

    winchMotor.set(speed);

  }

  public boolean getLimitSwitch(){

    return limitState;
    
  }







  

////////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {

    
    limitState = liftLimitSwitch.get(); 



    // This method will be called once per scheduler run
  }
}
