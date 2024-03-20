// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <LimelightHelpers.h>
#include <iostream>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "vector"
#include <numbers>
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/voltage.h>

using namespace ctre;

frc::Joystick m_stick_left{0};
//motors
phoenix::motorcontrol::can::TalonFX m_Talon_leftFront{1};
phoenix::motorcontrol::can::TalonFX m_Talon_leftBack{2};
phoenix::motorcontrol::can::TalonFX m_Talon_rightFront{3};
phoenix::motorcontrol::can::TalonFX m_Talon_rightBack{4};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  //double tx = LimelightHelpers::getTX("limelight");
  // frc::SmartDashboard::PutNumber("TX", tx);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

  //frc::SmartDashboard::PutNumber("TX", nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("tx").GetDouble(0.0));
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  
}

double getDistance() {
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");  

    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 19.3; 

    // distance from the target to the floor
    double goalHeightInches = 28; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
}

void Robot::TeleopPeriodic() {
  //double tx = LimelightHelpers::getTX("");
  double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  double speedLimiter = 0.6;
  if (tx < 0) {
    //turn right
    m_Talon_leftBack.Set(ControlMode::PercentOutput, speedLimiter);
    m_Talon_leftFront.Set(ControlMode::PercentOutput, speedLimiter);
    m_Talon_rightBack.Set(ControlMode::PercentOutput, -speedLimiter);
    m_Talon_rightFront.Set(ControlMode::PercentOutput, -speedLimiter);
    std::cout << "left" << "\n"; 
  }
  else {
    //turn left
     m_Talon_leftBack.Set(ControlMode::PercentOutput, -speedLimiter);
    m_Talon_leftFront.Set(ControlMode::PercentOutput, -speedLimiter);
    m_Talon_rightBack.Set(ControlMode::PercentOutput, speedLimiter);
    m_Talon_rightFront.Set(ControlMode::PercentOutput, speedLimiter);
    std::cout << "right" << "\n";
  }
  m_Talon_leftBack.ConfigOpenloopRamp(1);
  m_Talon_leftFront.ConfigOpenloopRamp(1);
  m_Talon_rightBack.ConfigOpenloopRamp(1);
  m_Talon_rightFront.ConfigOpenloopRamp(1);
  frc::SmartDashboard::PutNumber("TX", tx);
  frc::SmartDashboard::PutNumber("Distance", getDistance());
  std::cout << getDistance() << "\n";
  //frc::SmartDashboard::PutNumber("TX", nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetEntry("tx").GetDouble(0.0));
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
