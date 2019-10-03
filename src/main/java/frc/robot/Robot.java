/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 */
// current limit: 30 spike: 40

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.cscore.MjpegServer;
// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	AHRS m_navx = new AHRS(Port.kMXP);

	/** Hardware, either Talon could be a Victor */
	TalonSRX _leftMaster = new TalonSRX(3);
	TalonSRX _rightMaster = new TalonSRX(1);
	TalonSRX _leftSlave = new TalonSRX(4);
	TalonSRX _rightSlave = new TalonSRX(2);

	Joystick _gamepad = new Joystick(0);

	boolean toggleOn = false;
	boolean togglePressed = false;
	boolean m_2toggleOn = false;
	boolean m_2togglePressed = false;
	double m_valueH = 0;

	int m_autoMode = 2;

//	UsbCamera m_camera = new UsbCamera("Camera 0", 0);
//	MjpegServer m_server = new MjpegServer("server 0", 5800);

	@Override
	public void robotInit() {
		_leftMaster.configPeakCurrentLimit(40);
		_rightMaster.configPeakCurrentLimit(40);
		_leftMaster.configContinuousCurrentLimit(30);
		_rightMaster.configContinuousCurrentLimit(30);
		_leftSlave.configPeakCurrentLimit(40);
		_rightSlave.configPeakCurrentLimit(40);
		_leftSlave.configContinuousCurrentLimit(30);
		_rightSlave.configContinuousCurrentLimit(30);

		_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);
		_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,0,0);

		m_navx.reset();

//		String[] server = {"http://10.9.57.2:5800/stream.mjpg"};

/*		NetworkTableInstance.getDefault()
    		.getEntry("/CameraPublisher/Camera/streams")
    		.setStringArray(server);

		m_server.setSource(m_camera);
*/

	}
	
	@Override
	public void teleopInit(){

		/* Factory Default all hardware to prevent unexpected behaviour */
		_leftMaster.configFactoryDefault();
		_rightMaster.configFactoryDefault();
		_leftSlave.configFactoryDefault();
		_rightSlave.configFactoryDefault();

		/* Ensure motor output is neutral during init */
		_leftMaster.set(ControlMode.PercentOutput, 0);
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftSlave.set(ControlMode.Follower, _leftMaster.getDeviceID());
		_rightSlave.set(ControlMode.Follower, _rightMaster.getDeviceID());	
		
		/* Set Neutral mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		_leftSlave.setNeutralMode(NeutralMode.Brake);
		_rightSlave.setNeutralMode(NeutralMode.Brake);

		/* Configure output direction */
		_leftMaster.setInverted(false);
		_rightMaster.setInverted(true);
		_leftSlave.setInverted(false);
		_rightSlave.setInverted(true);

		m_navx.reset();
	}
	
	@Override
	public void teleopPeriodic() {

		/**

		// Gamepad processing
		m_valueH = ramp(_gamepad.getRawAxis(1), 0.1, m_valueH);
		double turn = _gamepad.getRawAxis(0);
		turn = -1 * deadband(turn);
		double m_2Turn = turn * _gamepad.getRawAxis(2);
		double m_2Forward = m_valueH;

    _rightMaster.set(ControlMode.PercentOutput, m_2Forward+m_2Turn);
	_leftMaster.set(ControlMode.PercentOutput, m_2Forward-m_2Turn);
	_rightSlave.set(ControlMode.PercentOutput, m_2Forward+m_2Turn);
	_leftSlave.set(ControlMode.PercentOutput, m_2Forward-m_2Turn);
	*/
//		updateToggle();

	}

	/** Deadband 5 percent, used on the gamepad */
	double deadband(double value) {
		/* Upper deadband */
		if (value >= +0.20 ) 
			return value-0.2;
		
		/* Lower deadband */
		if (value <= -0.20)
			return value+0.2;
		
		/* Outside deadband */
		return 0;
	}

	public void updateToggle(){
        if(_gamepad.getRawButton(3)){
            if(!togglePressed){
                toggleOn = !toggleOn;
                togglePressed = true;
            }
        }else{
            togglePressed = false;
		}
		
		if(_gamepad.getRawButton(1)){
            if(!m_2togglePressed){
                m_2toggleOn = !m_2toggleOn;
                m_2togglePressed = true;
            }
        }else{
            m_2togglePressed = false;
		}
	}

	public void robotPeriodic(){
		SmartDashboard.putNumber("Talon Left Encoder", _leftMaster.getSelectedSensorPosition());
		SmartDashboard.putNumber("Talon Right Encoder", _rightMaster.getSelectedSensorPosition());
		SmartDashboard.putNumber("Power Right", _rightMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Power Left", _leftMaster.getMotorOutputPercent());
		SmartDashboard.putNumber("Set Speed", _gamepad.getRawAxis(2));
		SmartDashboard.putNumber("Angle", m_navx.getAngle());
		SmartDashboard.putNumber("Y Speed", m_navx.getVelocityY());
	}

	public void autonomousPeriodic(){
		switch(m_autoMode){
			case 0:
				// Do nothing case

				break;
			case 1:
				// Lauren's Auto

				break;
			case 2:
				// Miles' Auto

				driveStraight(18, 90, .5);

				break;
			case 3:
				// Zach's Auto

				break;
		}
	}

	double ramp(double joystick, double ramp, double historicalValue) {
        double joystickdeadband = deadband(joystick);
        double throtle = _gamepad.getRawAxis(2);
         return historicalValue - (historicalValue - joystickdeadband * throtle)*ramp;
  }

  double getHeading(){
    return m_navx.getAngle();
  }
  public boolean driveStraight(int full_distance, double targetAngle, double speed){
	  double turn = 0;
	  double distance = full_distance * 54.324;

	  if(targetAngle - 0.5 > getHeading()){
		  turn = 0.05;
	  }
	  if(targetAngle + 0.5 < getHeading()){
		turn = -0.05;
	}
	  if(getDistance() < distance){
		_rightMaster.set(ControlMode.PercentOutput, -speed + turn);
		_leftMaster.set(ControlMode.PercentOutput, speed + turn);
		return false;
		  
	  }else{
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput, 0);
		return true;

	  }
	  

    
  }
  
  double getDistance(){
	  return (-_leftMaster.getSelectedSensorPosition() + _rightMaster.getSelectedSensorPosition())/2;
  }

  public void resetEncoders(){
	  _leftMaster.setSelectedSensorPosition(0);
	  _rightMaster.setSelectedSensorPosition(0);
  }



  }