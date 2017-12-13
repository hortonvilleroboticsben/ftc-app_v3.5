package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Block", group = "Test")
public class GlyphColor extends StateMachine_v5 {

    boolean isRunning = false;

    StateMachine_v5 driveMachine = new StateMachine_v5();

    @Override
    public void init(){
        super.init();

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){

//        if(snsColor.red() <= 1 && snsColor.blue() >= 3 && snsColor.green() <= 1 || isRunning){
//            initializeMachine(driveMachine);
//            if(next_state_to_execute(driveMachine)){
//                isRunning = true;
//                incrementState(driveMachine);
//            }
//
//            //ServoMove(driveMachine, srv, 1.0);
//            //Pause(driveMachine, 3000);
//            Drive(driveMachine, 1, .75);
//            //ServoMove(driveMachine, srv, 0.0);
//
//            if (snsColor.red() <= 0 && snsColor.blue() <= 0 && snsColor.green() <= 0 || isRunning){
//                initializeMachine(driveMachine);
//                if(next_state_to_execute(driveMachine)) {
//                    isRunning = true;
//                    incrementState(driveMachine);
//
//                    Turn(driveMachine, -1, .75);
//                    Turn(driveMachine, 2, .75);
//                    Turn(driveMachine, -1, .75);
//                }
//            }
//
//            if(next_state_to_execute(driveMachine)) {
//                incrementState(driveMachine);
//                driveMachine.state_in_progress = 1;
//                isRunning = false;
//            }
//
//
//        }
//
//        telemetry.addData("Red Value",snsColor.red());
//        telemetry.addData("Blue Value", snsColor.blue());
//        telemetry.addData("Green Value", snsColor.green());

    }

}
