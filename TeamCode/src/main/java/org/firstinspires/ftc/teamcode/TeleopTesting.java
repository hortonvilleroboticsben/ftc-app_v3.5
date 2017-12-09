package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.RobotLog;


@TeleOp(name = "TeleopTesting", group = "Test")
public class TeleopTesting extends StateMachine_v5 {

    ColorSensor snsColor;


    @Override
    public void init(){
        super.init();
        try {
            snsColor = hardwareMap.get(ColorSensor.class,"snsColor");
            snsColor.enableLed(true);
        } catch (Exception p_exception) {
            telemetry.addData("snsColor","disconnected");
            RobotLog.i(p_exception.getLocalizedMessage());
            snsColor = null;
        }
    }

    @Override
    public void loop(){

//        if(gamepad1.dpad_left) crS.setPower(1.0);
//        else if(gamepad1.dpad_right) crS.setPower(-1.0);
//        else crS.setPower(0.0);
//        telemetry.addData("Power", crS.getPower());
        /*
        telemetry.addData("red",snsColor.red());
        telemetry.addData("blue",snsColor.blue());
        telemetry.addData("green",snsColor.green());
        */
        telemetry.addData("", mtrFlip.getCurrentPosition());

    }

}