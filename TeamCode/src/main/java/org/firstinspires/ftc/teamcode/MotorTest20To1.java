package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;


//@TeleOp(name = "MotorTest20:1", group = "Test")
public class MotorTest20To1 extends OpMode
{

    DcMotor mtr20;

    @Override
    public void init()
    {

        mtr20 = hardwareMap.dcMotor.get("mtr20");
        gamepad1 = new Gamepad();

    }

    @Override
    public void loop()
    {

        mtr20.setPower(gamepad1.left_stick_y);

    }

}
