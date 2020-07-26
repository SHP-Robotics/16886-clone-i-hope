package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class GyroTest extends GyroFunctions{
    private int stage =0;
    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        switch(stage) {
            case 0:
                if (auto_drive(0.5, 5)){
                    reset_drive_encoders();
                    resetAngle();
                    stage++;
                }
                break;
            case 1:
                if (auto_mecanum(0.5, 5)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:
                if (auto_drive(-0.5, 5)) {
                    reset_drive_encoders();
                    resetAngle();
                    stage++;
                }
                break;
            case 3:
                if (auto_turn(1, 45)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 4:
                if (auto_drive(0.5, 5)) {
                    reset_drive_encoders();
                    resetAngle();
                    stage++;
                }
                break;
            default:
                break;
        }
    }




}
