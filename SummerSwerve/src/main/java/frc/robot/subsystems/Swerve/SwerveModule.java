/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class SwerveModule {

private String name;
private double gearRatio;

private WPI_TalonSRX driveMotor;
private WPI_TalonSRX steerMotor;

private boolean reverseEncoder = false;
private boolean reverseSteer = false;

    public SwerveModule(String name, WPI_TalonSRX drive, WPI_TalonSRX steer){
        this.name = name;
        driveMotor = drive;
        steerMotor = steer;
        gearRatio = 1.0;
    }

    public void stop(){
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void setSpeed(double speed){
        driveMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setAngle(double angle){
        if (reverseSteer){
            steerMotor.set(ControlMode.Position, -1 * angle * gearRatio);
        }
        else{ steerMotor.set(ControlMode.Position, 1 * angle * gearRatio); }
        //steerMotor.set(ControlMode.Position, (reverseSteer ? -1 : 1) * angle * gearRatio);
    }

    public void setEncoder(int postion){
        steerMotor.setSelectedSensorPosition(postion, 0, 10);
    }

    public WPI_TalonSRX getDrive(){
        return driveMotor;
    }

    public WPI_TalonSRX getSteer(){
        return steerMotor;
    }

    public void move(double speed, double angle){
        int encPosition = getEncPosition();
        angle = convertAngle(angle, encPosition);
        
        setSpeed(speed);
        if (speed != 0.0) { setAngle(angle); 
        }
    }

    //checks if the wheel is at a spot to reverse to get to desired direction
    public boolean shouldReverse(double wa, double encoderValue){
        double ea = SwerveUtil.convertEncoderValue(encoderValue, gearRatio);
		
		if(wa < 0)	wa += 1;
		
		double longDiff = Math.abs(wa - ea);
		
		double diff = Math.min(longDiff, 1.0-longDiff);
		
		if(diff > 0.25) { 
            return true; }
		else {return false; }
    }

    private double convertAngle(double angle, double encoderValue){
        double encPos = encoderValue / gearRatio;
        double temp = angle;
        temp += (int)encPos;

        encPos %= encPos;

        if((angle - encPos) >  0.5) temp -= 1;
		if((angle - encPos) < -0.5) temp += 1;
		
		return temp;
    }

    public void setEncPosition(int position){
		steerMotor.setSelectedSensorPosition(position, 0, 10);
    }
    
    public int getEncPosition(){
        int reverse;// = reverseEncoder ? -1 : 1;
        if(reverseEncoder){
            reverse = -1;
        }
        else{ reverse = 1; }
		return reverse * steerMotor.getSelectedSensorPosition(0);
    }

    public boolean isReverseEncoder(){
		return reverseEncoder;
	}
	
	public void setReverseEncoder(boolean reverseEncoder){
		this.reverseEncoder = reverseEncoder;
	}
	
	public void setReverseSteerMotor(boolean reverseSteer){
		this.reverseSteer = reverseSteer;
	}
}
