package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontaniers.MotorConfigurationType

class KotlinMecanumDriveManager private constructor(
    private val frontLeft: DcMotorEx,
    private val frontRight: DcMotorEx,
    private val backLeft: DcMotorEx,
    private val backRight: DcMotorEx,
    private val motors: Array<DcMotorEx>
) {
    
}
