#include "pros/adi.hpp"
#include <cmath>
#include "pros/rtos.hpp"
#include "robot_config.h"
#include "basic_functions.h"
#include "PID.h"
#include "odom.h"


void auto_skills()
{
    imu.set_heading(180);
    jemintaket();//FIRST HALF
    jeminmecht();
    jeminwingt();
    setintakespd(-100);
    setintake2spd(0);
    pros::delay(1000);
    moveForSec(90, true, 0.5);
    pros::delay(300);
    moveForSec(70, false, 0.1);
    moveleft(77);
    moveright(77);
    pros::delay(400);
    moveForSec(60, false, 0.2);
    moveleft(77);
    moveright(20);
    pros::delay(400);
    moveForSec(30, true, 0.3);
    moveForSec(60, false, 0.9);
    setintakespd(0);
    setintake2spd(-5);


    pidFrontWallGyro(25, 180, 0, 0.7, 60);
    pidTurnAbs(47, 10, 1);
    setintakespddiff(-100, 0);
    pidGyro(22, 47, 2, 100);


    pidGyro(22, 315, 1.5, 70, 1, 25, 1, 400);

    moveForSec(35, true, 0.4);
    jemintaket();
    setintakespddiff(80, 80);
    setintake2spd(25);
    pros::delay(300);
    setintakespddiff(70, 60);
    pros::delay(300);
    setintakespddiff(-100, -100);
    moveForSec(20, false, 0.1);
    setintakespddiff(70, 56);
    pros::delay(200);
    setintakespddiff(50, 45);
    pros::delay(2700);

    //jeminmecht();
    moveForSec(30, false, 0.3);
    pidwallGyro(24, 310, 90, 3);

    jeminloadert();
    jemintaket();
    pidTurnAbs(182, 1, 1);


    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(19, 180, 1);
    moveForSec(55, true, 0.5);
    moveForSec(10, true, 1.5);
    //DONE MATCHLOAD

    moveForSec(60, false, 0.1);
    jeminloadert();
    pidTurnRel(50, 2, 3, 60);
    setintakespddiff(-100, 0);
    setintake2spd(0);

    pidGyro(-130, 180, 7, 90, 30, 100, 2, 300);
    pidwallGyro(37, 180, 180, 2, 100, 3);

    pidTurnAbs(270, 10, 0.8);
    pidWallMove(24.5, 0.5, 90, 2);
    pidTurnAbs(0, 2, 0.7);

    moveForSec(60, false, 0.6);

    skillscore();
    jeminloadert();
    moveForSec(60, false, 1.6); //FIRST SCORE DONE
    imu.set_heading(0);

    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(33, -4, 2, 80, 3);
    moveForSec(55, true, 1.8);
    pidGyro(-40, 6, 2);

    skillscore();
    moveForSec(30, false, 1.6); //SECOND SCORE DONE
    imu.set_heading(0);
    jeminloadert();
    setintakespd(0);
    setintake2spd(0);



    //TO CLEAR PARK
    //pidMoveold(7, 0.5, 2, 70); 
    //pidTurnAbs(270, 5, 0.6);
    //pidGyro(96, 270, 5, 75); //91 PATH

    //imu.set_heading(0);

    pidmove(19, 1, 2, 100);
    pidTurnAbs(-35, 5, 1);
    pidGyro(40, 295, 2, 100, 11, 25, 2, 100);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(-5);
    moveForSec(65, true, 0.2);
    jeminloadert();
    moveForSec(65, true, 1.6);



    pidFrontWallGyro(20, 235, 270, 1.8, 80);
    pidTurnAbs(2, 3, 1);

    pidGyro(-29, 2, 1.4, 90);
    skillscore();
    imu.set_heading(0);
    jeminloadert();
    moveForSec(50, false, 0.5); //PARK SCORE DONE

    setintake2spd(0);
    setintakespd(-100);
    pidGyro(34, 5, 2);
    moveForSec(45, true, 2.2);
    //3 MATCHLOAD DONE



    moveForSec(60, false, 0.13);
    jeminloadert();
    pidTurnRel(45, 2, 3, 60);

    setintakespddiff(-100, 0);
    setintake2spd(0);


    pidGyro(-130, 0, 7, 90, 30, 100, 2, 200);
    pidwallGyro(37, 0, 0, 2, 100, 3);

    pidTurnAbs(90, 10, 1);
    pidWallMove(24.5, 0.5, 270, 2);
    pidTurnAbs(178, 2, 0.7);


    moveForSec(50, false, 0.8);


    jeminloadert();
    skillscore();
    moveForSec(50, false, 1.8); //THIRD SCORE DONE
    imu.set_heading(180);

    setintakespd(-100);
    setintake2spd(-5);
    pidGyro(34, 176, 2);
    moveForSec(55, true, 1.7);
    pidGyro(-40, 186, 2);


    skillscore();
    jeminloadert();
    moveForSec(30, false, 0.5); //FOURTH SCORE DONE
    setintakespd(-50);
    setintake2spd(-50);
    moveForSec(30, false, 1.3);

    setintakespd(-100);
    setintake2spd(0);
    pidmove(19, 1, 2, 100);
    pidTurnAbs(145, 5, 1);
    pidGyro(40, 110, 2, 100, 11, 25, 2, 100);
    setintakespd(-100);
    setintake2spd(-5);
    moveForSec(70, true, 0.75);
    //*/
}


void Right_4ball()
{
    imu.set_heading(0);

    jemintaket();

    setintakespddiff(-100, -25);
    setintake2spd(0);
    pidGyro(22, 23, 1.2, 100, 3, 40, 1, 1000 ,1.5);
    jeminloadert();
    moveForSec(30, true, 0.2);


    pidGyro(36, 125, 2, 100, 1, 25, 5, 20, 1.5);
    jeminloadert();

    pidTurnAbs(180, 5, 0.8);
    moveForSec(85, false, 0.4);
    score_on = true;
    //jeminloadert();
    moveForSec(50, false, 1);
    //jeminloadert();


    pidGyro(13, 135, 2, 100, 1, 25, 1, 15, 1.5);
    score_on = false;
    jeminwing.set_value(false);
    pros::delay(1);
    pidGyro(0, 183, 0.9, 100, 1, 25, 1, 15, 1.5);
    pidmove(-31, 1, 2);
    setintakespd(0);
    setintake2spd(0);
    moveright(-24);
    pros::delay(1000000);
}


void Left_4ball()
{
    imu.set_heading(0);

    jemintaket();
    jeminmecht();

    setintakespddiff(-100, -100);
    setintake2spd(0);
    pidGyro(22, -23, 1.2, 100, 3, 40, 1, 1000);
    jeminloadert();
    moveForSec(30, true, 0.2);


    pidGyro(-28, 70, 2, 100, 1, 25, 5, 30);
    //pros::delay(2000);

    moveright(-100);
    pros::delay(500);

    score_on = true;
    moveright(-100);
    pros::delay(400);
    moveForSec(60, false, 0.7);
    jeminloadert();
    pidGyro(12, 135, 2, 100, 1, 25, 5, 30, 4);
    score_on = false;
    jeminwing.set_value(false);
    //pros::delay(1000);
    pidGyro(-33, 180, 4, 100, 1, 25, 15, 5, 1.3);
    setintakespd(0);
    setintake2spd(0);
    //pidmove(-31, 1, 2);
    //pidTurnRel(20, 2, 1);
    moveright(-24);
    pros::delay(1000000);
}



void Right_3_4()
{
    imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27, 0.5, 90, 2.5, 100);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 182, 0.8, 100);
    moveForSec(50, true, 0.75);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 182, 1.5, 100);
    jeminmech.set_value(true);
    score_on = true;

    moveForSec(50, false, 1.35);
    score_on = false;
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(0);
    setintakespddiff(-100, 0);

    pidGyro(20, 295, 2, 100, 1, 25, 5, 60);
    jeminloadert();
    moveForSec(30, true, 0.2);
    pidTurnAbs(310, 10, 0.8, 100);
    jeminloadert();
    pidGyro(22, 320, 1.3, 80, 1, 25, 1);
    setintake2spd(0);

    jemintaket();
    setintakespd(60);
    moveForSec(30, true, 1);
    setintakespd(0);
    //jemintaket();

    jeminwingt();

    pidGyro(-22, 270, 3, 100, 1, 25, 1, 1000);
    pidGyro(-13, 10, 1.5, 100, 1, 25, 2, 500);

    jeminwingt();
    jemintaket();
    //pros::delay(200);
    moveForSec(70, true, 0.4);
    pidTurnRel(-40, 1, 1, 100);
    moveright(24);
    pros::delay(1000000);
}


void Left_3_4()
{
    imu.set_heading(0);

    jemintaket();

    setintakespddiff(-100, -100);
    setintake2spd(0);
    pidGyro(22, -23, 1.2, 100, 3, 40, 1, 1000);
    jeminloadert();
    moveForSec(30, true, 0.2);


    pidTurnAbs(75, 10, 0.7);
    pidwallGyro(25, 60, 270, 1.7, 100, 1);
    //pros::delay(2000);

    pidTurnAbs(178, 3, 0.7);
     moveForSec(70, false, 0.3);
    score_on = true;
    moveForSec(60, false, 1);

    score_on = false;
    setintakespd(-100);
    setintake2spd(0);
    pidGyro(34, 177, 1, 100);
    moveForSec(40, true, 0.7);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    storing = true;
    jeminmech.set_value(false);
    pidGyro(-66, 226, 2, 100, 10, 25, 1, 41);
    //pros::delay(1000);
    jeminmech.set_value(true);
    moveForSec(60, false, 0.6);
    setintakespd(30);
    setintake2spd(0);
    moveForSec(50, false, 0.1);
    storing = false;
    setintakespddiff(-100, -70);
    setintake2spd(70);
    moveForSec(40, false, 1);
    setintakespd(0);
    setintake2spd(0);
    jemintaket();


    pidGyro(37, 276, 2.5, 100, 1, 25, 1, 100);
    //pidGyro(16, 170, 1.5, 100, 1, 25, 1, 100);
    //jemintaket();

    pidTurnRel(130, 1, 1, 75);
    moveleft(30);
    //moveForSec(50, true, 100);
}



void SAWP()
{
    imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    moveForSec(40, true, 0.3);

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27, 0.5, 90, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 184, 0.8, 100);
    moveForSec(50, true, 0.75);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 183, 1.5, 100, 1, 25, 15, 15, 1.5);
    jeminmech.set_value(true);
    score_on = true;

    moveForSec(50, false, 1);
    score_on = false;
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(0);
    setintakespddiff(-100, 0);

    pidGyro(23, 297, 2, 100, 5, 50, 1, 70);
    jeminloadert();
    moveForSec(40, true, 0.2);

    storing = true;
    jeminmech.set_value(false);

    
    jeminloadert();
    pidGyro(41, 270, 2.5, 100, 10, 100, 1);
    jeminloadert();
    moveForSec(50, true, 0.2);

    // pidmove(48, 0.5, 3, 100);
    pidTurnAbs(215, 15, 0.5, 100);
    pidFrontWallGyro(21, 235, 270, 1.4, 100);


    pidTurnAbs(180, 10, 0.7, 100);


    pidGyro(-27, 179, 1, 100, 1, 25, 1);
    jeminmech.set_value(true);
    setintake2spd(-100);
    setintakespd(-100);
    moveForSec(60, false, 1.25);
    setintake2spd(0);

    pidGyro(34, 179, 1, 100);
    moveForSec(50, true, 0.7);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    storing = true;
    jeminmech.set_value(false);
    pidGyro(-60, 227.5, 2, 100, 10, 25, 1, 60);
    jeminmech.set_value(true);
    setintakespd(30);
    setintake2spd(0);
    moveForSec(50, false, 0.4);
    //moveForSec(40, false, 0.3);
    storing = false;
    setintakespddiff(-100, -70);
    setintake2spd(80);
    moveForSec(30, false, 1);
}



void right_7()
{
    imu.set_heading(0);

    jemintaket();

    setintakespddiff(-100, -100);
    setintake2spd(0);
    pidGyro(23, 23, 1.2, 100, 3, 40, 1, 10000 ,1);
    jeminloadert();
    moveForSec(30, true, 0.2);



    pidTurnAbs(122, 10, 2);
    pidGyro(44, 179, 2, 100, 1, 25, 2, 1000000, 1, 50);


    moveForSec(50, true, 0.75);
    
    pidGyro(-40, 181, 2, 100, 1, 25, 1);

    score_on = true;
    moveForSec(50, false, 1.4);
    jeminloadert();
    pidGyro(9.5, 145, 2, 100, 1, 25, 5, 30, 4);
    score_on = false;
    jeminwing.set_value(false);
    //pros::delay(1000);
    pidGyro(-17, 200, 4, 100, 1, 25, 15, 15, 1.1);
    setintakespd(0);
    setintake2spd(0);
    pros::delay(3000);
    pidmove(-12, 1, 2);
    //pidTurnRel(20, 2, 1);
    moveright(-24);
    pros::delay(1000000);

}


void left_7()
{
    imu.set_heading(0);

    jemintaket();

    setintakespddiff(-100, -100);
    setintake2spd(0);
    pidGyro(23, -23, 1.2, 100, 3, 40, 1, 10000 ,1);
    jeminloadert();
    moveForSec(30, true, 0.2);

    pidTurnAbs(-125, 10, 2);
    pidGyro(44, 180, 2, 100, 1, 25, 2, 1000000, 1, 45);

    moveForSec(50, true, 0.75);
    
    pidGyro(-44, 180, 2, 100, 1, 25, 1);



    score_on = true;
    moveForSec(50, false, 1.6);
    jeminloadert();
    pidGyro(12, 135, 2, 100, 1, 25, 5, 30, 4);
    score_on = false;
    jeminwing.set_value(false);
    //pros::delay(1000);
    pidGyro(-17, 180, 4, 100, 1, 25, 15, 15, 1.1);
    setintakespd(0);
    setintake2spd(0);
    pros::delay(3000);
    pidmove(-12, 1, 2);
    //pidTurnRel(20, 2, 1);
    moveright(-24);
    pros::delay(1000000);
}
