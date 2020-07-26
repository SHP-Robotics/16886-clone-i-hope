package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueBuildZoneParkOnly extends BaseRobot{
    private int stage = 0;
    @Override
    public void init () {
        super.init();
        claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
    }

    @Override
    public void start (){
        super.start();
        timer.reset();
    }

    @Override
    public void loop (){
        super.loop();
        switch(stage) {
            case 0:
                if (timer.seconds()>=25) {
                    stage++;
                }
                break;
            case 1:
                if (auto_mecanum(5, 1)){
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default:
                break;
        }

    }
}
