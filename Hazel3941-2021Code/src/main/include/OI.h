/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "frc/Joystick.h"
#include "RobotMap.h"
#include "frc/DigitalInput.h"

class OI {
  public:
    OI();
    frc::Joystick* DriverController;
    frc::Joystick* OperatorController;
    frc::DigitalInput dio_indexer_1a{DIO_INDEXER_1A};
    frc::DigitalInput dio_indexer_1b{DIO_INDEXER_1B};
    frc::DigitalInput dio_indexer_2{DIO_INDEXER_2};
    frc::DigitalInput dio_indexer_3{DIO_INDEXER_3};
    frc::DigitalInput dio_indexer_4{DIO_INDEXER_4};
    frc::DigitalInput dio_indexer_5{DIO_INDEXER_5};
    frc::DigitalInput dio_shooter_1{DIO_SHOOTER_1};
    frc::DigitalInput dio_shooter_2{DIO_SHOOTER_2};

};
