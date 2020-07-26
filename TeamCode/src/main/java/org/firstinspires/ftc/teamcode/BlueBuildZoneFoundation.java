package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueBuildZoneFoundation extends BaseRobot{
    private int stage = 0;
    @Override
    public void init (){
        super.init();
        //claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
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
                arm2(-1);
                stage++;
                break;
            case 1:
                stage++;
                break;
            case 2:
                if (auto_mecanum(-0.5, 5)) {
                    reset_drive_encoders();
                    arm2(0);
                    stage++;
                }
                break;
            case 3:
                if (auto_drive(0.5, 20)) {
                    reset_drive_encoders();
                    base_mover(ConstantVariables.K_BASE_SERVO_RIGHT_DOWN, ConstantVariables.K_BASE_SERVO_LEFT_DOWN);
                    timer.reset();
                    stage++;
                }
                break;
            case 4:
                if (timer.seconds()>1) {
                    stage++;
                }
                break;
            case 5:
                if (auto_drive(-0.5, 20)) {
                    reset_drive_encoders();
                    base_mover(ConstantVariables.K_BASE_SERVO_RIGHT_UP, ConstantVariables.K_BASE_SERVO_LEFT_UP);
                    timer.reset();
                    stage++;
                }
                break;
            case 6:
                if (timer.seconds()>1){
                    stage++;
                }
                break;
            case 7:
                if (auto_drive(-0.5, 1)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 8:
                if (auto_mecanum(0.5, 25)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 9:
                stage++;
                break;
            default:
                break;
        }
    }
}
