#include <iostream>
#include <memory>
#include <string>
#include "WPILib.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <XboxController.h>

#define CAN_ID_LEFTMASTER	1
#define CAN_ID_LEFTSLAVE	2
#define CAN_ID_RIGHTSLAVE	3
#define CAN_ID_RIGHTMASTER	4

#define MAX_LEFT_RPM	455.0
#define MAX_RIGHT_RPM	455.0

#include <CANTalon.h>

class Robot: public frc::IterativeRobot
{

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

private:
	CANTalon _talonMasterLf {CAN_ID_LEFTMASTER};
	CANTalon _talonSlaveLf  {CAN_ID_LEFTSLAVE};
	CANTalon _talonMasterRt{CAN_ID_RIGHTMASTER};
	CANTalon _talonSlaveRt  {CAN_ID_RIGHTSLAVE};
	frc::XboxController	_joy{0};
	std::string _sb;
	uint32_t _loops = 0;
	bool _lastButtonA = false;
	bool _lastButtonB = false;
	bool _displayToggle = true;
	double targetPositionRotations;

public:
	void RobotInit()
	{
		// left drive
		//
		// choose the sensor
		_talonMasterLf.SetFeedbackDevice(CANTalon::QuadEncoder);
		_talonMasterLf.ConfigEncoderCodesPerRev(2048);
		_talonMasterLf.SetSensorDirection(false);
		// peak and nominal outputs, 12V means full
		_talonMasterLf.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		_talonMasterLf.ConfigPeakOutputVoltage(+12.0f, -12.0f);
		// closed loop gains in slot 0
		_talonMasterLf.SelectProfileSlot(0);
		_talonMasterLf.SetF(0.1561);
		_talonMasterLf.SetP(0.01);
		_talonMasterLf.SetI(0.001);
		_talonMasterLf.SetD(0.0);
		_talonMasterLf.SetCloseLoopRampRate(0.0);
		_talonMasterLf.SetIzone(0.0);
		// slave
		_talonSlaveLf.SetControlMode(CANSpeedController::kFollower);
		_talonSlaveLf.Set(CAN_ID_LEFTMASTER);

		// right Drive System
		//
		// choose the sensor
		_talonMasterRt.SetFeedbackDevice(CANTalon::QuadEncoder);
		_talonMasterRt.ConfigEncoderCodesPerRev(2048);
		_talonMasterRt.SetSensorDirection(false);
		// set the peak and nominal outputs, 12V means full
		_talonMasterRt.ConfigNominalOutputVoltage(+0.0f, -0.0f);
		_talonMasterRt.ConfigPeakOutputVoltage(+12.0f, -12.0f);
		// closed loop gains in slot 0
		_talonMasterRt.SetF(0.1561);
		_talonMasterRt.SetP(0.01);
		_talonMasterRt.SetI(0.001);
		_talonMasterRt.SetD(0.0);
		_talonMasterRt.SetCloseLoopRampRate(0.0);
		_talonMasterRt.SetIzone(0.0);
		// slave
		_talonSlaveRt.SetControlMode(CANSpeedController::kFollower);
		_talonSlaveRt.Set(CAN_ID_RIGHTMASTER);
	}

	bool _rightSideSelect;
	bool _sideSelectPrev;
	CANTalon* _pTestTalon;
	double _maxRPM;

	void TeleopInit()
	{
		_rightSideSelect = false;;
		_sideSelectPrev = !_rightSideSelect;
		_talonMasterRt.Set(0);
		_talonMasterLf.Set(0);
		_maxRPM = 0.0;
		_pTestTalon = &_talonMasterLf;
	}

	void TeleopPeriodic()
	{
		bool voltageMode = _joy.GetAButton();
		bool _sideSelect = _joy.GetBumper(frc::GenericHID::kRightHand);

		if (_sideSelect != _sideSelectPrev)
		{
			// stop current Talon under test
			_pTestTalon->Set(0);
			// change test Talon based on switch
			_pTestTalon     = (_sideSelect) ? &_talonMasterRt : &_talonMasterLf;
			_maxRPM 	    = (_sideSelect) ? MAX_RIGHT_RPM : MAX_LEFT_RPM;
			_sideSelectPrev = _sideSelect;
		}

		double stickValue = _joy.GetY(frc::GenericHID::kLeftHand);
		if (!_sideSelect)
			stickValue = - stickValue;
		double motorOutput = _pTestTalon->GetOutputVoltage() / _pTestTalon->GetBusVoltage();

		// prepare line to print
		double speed  = _pTestTalon->GetSpeed();
		double targetSpeed = 0.0;
		int encVel = _pTestTalon->GetEncVel();
		int error = 0;

		if (fabs(stickValue) < .05)
		{
			_pTestTalon->Set(0);
		}
		else
		{
			if (voltageMode)
			{
				 // Percent voltage mode
				_pTestTalon->SetControlMode(CANSpeedController::kPercentVbus);
				_pTestTalon->Set(stickValue);
			}
			else
			{
				// Speed mode - button just pressed
				targetSpeed = stickValue * _maxRPM;
				_pTestTalon->SetControlMode(CANSpeedController::kSpeed);
				_pTestTalon->Set(targetSpeed);

				error = _pTestTalon->GetClosedLoopError();
			}

			// print every ten loops, printing too much too fast is generally bad for performance
			if (++_loops >= 10)
			{
				_loops = 0;
				printf("in: % 6.3f out: % 6.3f spd: % 6.3f vel: % 6i err: %+4i trg: % 6.3f\n", stickValue, motorOutput, speed, encVel, error, targetSpeed);

			}
		}

	}

	void AutonomousInit() override
	{
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom)
		{
			// Custom Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if (autoSelected == autoNameCustom)
		{
			// Custom Auto goes here
		}
		else
		{
			// Default Auto goes here
		}
	}


	void TestPeriodic()
	{
		lw->Run();
	}

};
START_ROBOT_CLASS(Robot);
