package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "270 Testing", group = "Testing")
public class Servo270_Test extends StateMachine_v5 {
    Servo srv270;
    int pos = 0;

    @Override
    public void init() {
        super.init();
        srv270 = addServo(0.0, "srv270");
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        set_position(srv270, pos/270.);
        pos+= !(gamepad1.dpad_up || gamepad1.dpad_down) ? 0 : gamepad1.dpad_up ? 1 : -1;
        pos = (pos > 270 || pos < 0) ? (pos > 270) ? 270 : 0 : pos;

        telemetry.addData("pos", pos);
    }
}
