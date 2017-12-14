package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//@TeleOp(name = "IMUTesting", group = "Test")
public class IMUTesting extends StateMachine_v5 {


    Orientation axes;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        IMUnav.startAccelerationIntegration(new Position(), new Velocity(DistanceUnit.INCH, 0, 0, 0, 50), 50);
    }

    @Override
    public void loop() {

        axes = IMUnav.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        if(Math.abs(axes.secondAngle) > 2){
            set_drive_power((axes.secondAngle < 0) ? 0.2 : -0.2, (axes.secondAngle < 0) ? 0.2 : -0.2);
        }else{
            if(Math.abs(axes.firstAngle) > 3){
                set_drive_power((axes.firstAngle < 0) ? 0.1 : -0.1, (axes.firstAngle < 0) ? -0.1 : 0.1);
            }else {
                set_drive_power(0.0, 0.0);
            }
        }

        telemetry.addData("IMU Velocity", IMUnav.getVelocity().xVeloc);
        telemetry.addData("IMU X Rotation", axes.firstAngle);
        telemetry.addData("IMU Y Rotation", axes.secondAngle);
        telemetry.addData("DecisionIf", Math.abs(axes.secondAngle) > 2);
        telemetry.addData("DecisionPow", (axes.secondAngle > 0));
        telemetry.addData("lMotor", mtrLeftDrive.getDirection());
        telemetry.addData("rMotor", mtrRightDrive.getDirection());
    }
}
