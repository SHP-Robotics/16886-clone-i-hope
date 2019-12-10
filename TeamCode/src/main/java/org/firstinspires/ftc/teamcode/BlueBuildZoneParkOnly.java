package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBuildZoneParkOnly extends BaseRobot{
    private int stage = 0;
    @Override
    public void init () {
        super.init();
        claw_open(3);
    }

    @Override
    public void start (){
        super.start();
    }

    @Override
    public void loop (){
        super.loop();
        switch(stage) {
            case 0:
                if (auto_mecanum(-1, 40)){
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }

    }
}
