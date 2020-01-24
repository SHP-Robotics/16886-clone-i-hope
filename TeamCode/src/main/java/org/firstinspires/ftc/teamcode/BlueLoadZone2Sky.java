package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueLoadZone2Sky extends BaseRobot {
    private int stage = 0;
    private double block_distance;
    private boolean wall_skystone;

    @Override
    public void init() {
        super.init();
        claw(ConstantVariables.K_CLAW_SERVO_OPEN);
    }

    @Override
    public void start() {
        super.start();
        color_sensor.enableLed(true);
    }

    @Override
    public void loop() {
        super.loop();
        switch (stage) {
            case 0:
                if (auto_drive(0.4, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:
                if (auto_mecanum(0.4, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:
                if (detect_line() || detect_stone()) {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                } else {
                    tankanum_drive(-0.25, -0.25, 0);
                }
                break;
            case 3:
                stage++;
                break;
            case 4:
                //moves right until no stone
                if (detect_stone()) {
                    tankanum_drive(0, 0, -0.5);
                } else {
                    if (auto_mecanum(0.5, -1)) {
                        block_distance = get_right_front_drive_motor_enc() / ConstantVariables.K_PPIN_DRIVE;
                        reset_drive_encoders();
                        claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                        timer = new ElapsedTime();
                        stage++;
                    }

                }
                break;
            case 5:
                if (timer.seconds()>0.25) {
                    stage++;
                }
                break;
            case 6:
                //moves forward
                if (auto_drive(1, 14)) {
                    reset_drive_encoders();
                    timer = new ElapsedTime();
                    stage++;
                }
                break;
            case 7:
                //closes claw
                claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                if (timer.seconds() > 0.5) {
                    stage++;
                }
                break;
            case 8:
                stage++;
            case 9:
                //moves back
                if (auto_drive(-1, 20)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 10:
                //mecanums right to build site
                if (auto_mecanum(-1, 80 - block_distance)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 11:
                //opens claw
                claw(ConstantVariables.K_CLAW_SERVO_OPEN);
                stage++;
                break;
            case 12:
                //moves back
                if (auto_drive(-1, 3)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 13:
                if (wall_skystone) {
                    stage = 31;
                } else {
                    stage++;
                }
                break;
            case 14:
                //goes to next stone
                if (auto_mecanum(1, 80 - block_distance - 24)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 15:
                //moves forward
                if (auto_drive(1, 10)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 16:
                if (detect_line() || detect_stone()) {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                } else {
                    tankanum_drive(-0.50, -0.50, 0);
                }
                break;
            case 17:
                if (detect_stone()) {
                    tankanum_drive(0, 0, -0.5);
                } else {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 18:
                if (auto_drive(1, 15)) {
                    reset_drive_encoders();
                    timer = new ElapsedTime();
                    stage++;
                }
                break;
            case 19:
                claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                if (timer.seconds() > 0.5) {
                    stage++;
                }
                break;
            case 20:
                if (auto_drive(-1, 25)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 21:
                if (auto_mecanum(-1, 65)) {
                    reset_drive_encoders();
                    timer = new ElapsedTime();
                    stage++;
                }
                break;
            case 22:
                claw(ConstantVariables.K_CLAW_SERVO_OPEN);
                if (timer.seconds() > 0.75) {
                    stage++;
                }
                break;
            case 23:
                if (auto_drive(-1, 5)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 24:
                if (auto_mecanum(1, 7)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 30:
                wall_skystone = true;
                if (detect_stone()) {
                    stage = 4;
                } else {
                    tankanum_drive(0, 0, -0.5);
                }
                break;
            case 31:
                if (auto_mecanum(1, 80 - block_distance - 24)) {
                    reset_drive_encoders();
                    stage = 18;
                }
                break;
            default:
                break;
        }
    }
}