package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedLoadZone2Sky extends BaseRobot {
    private int stage = 0;
    private double block_distance;
    //private boolean wall_skystone;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        color_sensor.enableLed(true);
        claw(ConstantVariables.K_CLAW_SERVO_OPEN);
        arm2(-1);
    }

    @Override
    public void loop() {
        super.loop();
        switch (stage) {
            case 0:
                if (timer.seconds()>0.75) {
                    arm2(0);
                    stage++;
                }
                break;
            case 1:
                if (auto_drive(0.35, 14)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:
                if (auto_mecanum(-0.4, 15)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 3:
                if (detect_line() || detect_stone()) {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                } else {
                    tankanum_drive(-0.25, -0.25, 0);
                }
                break;
            case 4:
                //moves right until no stone
                if (detect_stone()) {
                    tankanum_drive(0, 0, 0.2);
                } else {
                    tankanum_drive(0, 0, 0);
                    block_distance = get_right_front_drive_motor_enc() / ConstantVariables.K_PPIN_DRIVE;
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 5:
                stage++;
                break;
            case 6:
                //moves forward
                if (auto_drive(0.75, 3)) {
                    reset_drive_encoders();
                    arm2(1);
                    timer.reset();
                    stage++;
                }
                break;
            case 7:
                //closes claw
                claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                if (claw_servo.getPosition()<=ConstantVariables.K_CLAW_SERVO_CLOSED) {

                    stage++;
                }
                break;
            case 8:
                if (claw_servo.getPosition()<=ConstantVariables.K_CLAW_SERVO_CLOSED) {
                    arm2(-1);
                    stage++;
                }
                break;
            case 9:
                //moves back
                if (auto_drive(-0.75, 8)) {
                    //change idea: 4 in?
                    reset_drive_encoders();
                    arm2(0);
                    stage++;
                }
                break;
            case 10:
                //mecanums right to build site
                if (auto_mecanum(0.35, 50 - block_distance)) {
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
                if (auto_mecanum(-0.35, 5)) {
                    claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                    arm2(1);
                    timer.reset();
                    stage++;
                }
                break;
            case 13:
                if (timer.seconds()>1.5) {
                    arm2(0);
                    stage++;
                }
                break;
            case 14:
                //goes to next stone
                if (auto_mecanum(-0.35, (40 - block_distance))) {
                    reset_drive_encoders();
                    arm2(-1);
                    stage++;
                }
                break;
            case 15:
                //moves back
                if (auto_drive(-0.5, 2)) {
                    reset_drive_encoders();
                    claw(ConstantVariables.K_CLAW_SERVO_OPEN);
                    arm2(0);
                    stage++;
                }
                break;
            case 16:
                //moves forward
                if (auto_drive(0.5, 6)) {
                    reset_drive_encoders();
                    stage++;
                }
            case 17:
                if (detect_line() || detect_stone()) {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                } else {
                    tankanum_drive(-0.25, -0.25, 0);
                }
                break;
            case 18:
                if (detect_stone()) {
                    tankanum_drive(0, 0, 0.25);
                } else {
                    tankanum_drive(0, 0, 0);
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 19:
                if (auto_drive(0.5, 3)) {
                    reset_drive_encoders();
                    timer = new ElapsedTime();
                    stage++;
                }
                break;
            case 20:
                claw(ConstantVariables.K_CLAW_SERVO_CLOSED);
                if (claw_servo.getPosition()<=ConstantVariables.K_CLAW_SERVO_CLOSED) {
                    stage++;
                }
                break;
            case 21:
                if (auto_drive(-0.5, 7)) {
                    reset_drive_encoders();
                    arm2(1);
                    stage++;
                }
                break;
            case 22:
                if (auto_mecanum(0.4, 40-block_distance)) {
                    reset_drive_encoders();
                    timer = new ElapsedTime();
                    arm2(0);
                    stage++;
                }
                break;
            case 23:
                claw(ConstantVariables.K_CLAW_SERVO_OPEN);
                if (timer.seconds() > 0.75) {
                    arm2(-1);
                    timer.reset();
                    stage++;
                }
                break;
            case 24:
                if (timer.seconds()>1) {
                    timer.reset();
                    arm2(0);
                    stage++;
                }
                break;
            case 25:
                if (auto_mecanum(-0.4, 10)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            /*
            case 30:
                wall_skystone = true;
                if (detect_stone()) {
                    stage = 4;
                } else {
                    tankanum_drive(0, 0, 0.5);
                }
                break;
            case 31:
                if (auto_mecanum(-1, 80 - block_distance - 24)) {
                    reset_drive_encoders();
                    stage = 18;
                }
                break;
             */
            default:
                break;
        }
    }
}