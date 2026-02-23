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
        pros::delay(500);
        moveForSec(70, true, 0.1);
        moveForSec(60, false, 0.1);
        moveForSec(70, true, 0.1);
        moveForSec(60, false, 0.1);
        pros::delay(700);
        moveleft(70);
        moveright(65);
        pros::delay(600);
        //moveForSec(70, true, 0.6);
        moveForSec(60, false, 0.1);
        moveForSec(70, true, 0.1);
        moveForSec(60, false, 0.1);
        moveForSec(70, true, 0.1);
        pros::delay(400);
        moveForSec(30, true, 0.3);
        moveForSec(60, false, 0.8);
        setintakespd(0);
        setintake2spd(-5);


        pidFrontWallGyro(25, 180, 0, 0.7, 60);
        pidTurnAbs(47, 10, 1);
        setintakespddiff(-100, 0);
        pidGyro(22, 47, 2, 100);


        pidGyro(22, 315, 1.5, 70, 1, 25, 1, 400);

        moveForSec(30, true, 0.35);
        jemintaket();
        setintakespddiff(90, 80);
        setintake2spd(25);
        pros::delay(500);
        setintakespddiff(70, 56);
        pros::delay(500);
        setintakespddiff(-65, -65);
        pros::delay(100);
        setintakespddiff(70, 56);
        pros::delay(200);
        setintakespddiff(50, 50);
        pros::delay(1800);

        //jeminmecht();
        pidwallGyro(24.5, 310, 90, 3);

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

        pidGyro(-99, 180, 5.3, 90, 30, 100, 2, 300);
        pidwallGyro(37, 180, 180, 1, 100, 3);

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
        setintakespd(-100);
        setintake2spd(-5);
        moveForSec(55, true, 2.1);  // 60, 1.95


        pidFrontWallGyro(20, 235, 270, 1.8, 80);
        pidTurnAbs(2, 3, 1);

        pidGyro(-27, 2, 1.4, 90);
        skillscore();
        imu.set_heading(0);
        jeminloadert();
        moveForSec(30, false, 0.5); //PARK SCORE DONE

        setintake2spd(0);
        setintakespd(-100);
        pidGyro(34, 5, 2);
        moveForSec(45, true, 2.2);
        //3 MATCHLOAD DONE



        moveForSec(60, false, 0.13);
        jeminloadert();
        pidTurnRel(50, 2, 3, 60);

        setintakespddiff(-100, 0);
        setintake2spd(0);


        pidGyro(-99, 5, 6, 90, 30, 100, 5, 100);
        pidwallGyro(37, 0, 0, 0.5, 100, 3);

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
        moveForSec(30, false, 1.8); //FOURTH SCORE DONE

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


    //Right 4 Ball 
    /*
    imu.set_heading(8.5);

    jemintaket();
    jeminwingt();

    setintakespddiff(-100, -25);
    pidMoveold(19.3, 0.5, 200, 100);
    jeminloadert();
    pidMoveold(9.7, 0.5, 200, 100);
    //setintakespd(0);
    // pros::delay(100);

    pidGyro(32, 115, 3, 100);
    std::cout << "gheigh" << std::endl;

    // pidTurnAbs(180, 1, 0.3);
    pidGyro(0, 180, 1, 100);
    moveForSec(75, false, 0.4);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1400);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    //*/


    //Left 4 Ball 
    /*
    imu.set_heading(351.5);

    jemintaket();
    jeminwingt();

    setintakespddiff(-100, -25);
    pidMoveold(19.3, 0.5, 200, 100);
    jeminloadert();
    pidMoveold(9.7, 0.5, 200, 100);
    //setintakespd(0);
    // pros::delay(100);

    pidGyro(35, -120, 3, 100);
    std::cout << "gheigh" << std::endl;

    // pidTurnAbs(180, 1, 0.3);
    pidGyro(0, 180, 1, 100);
    moveForSec(75, false, 0.4);
    setintakespd(-100);
    setintake2spd(-100);
    jeminmecht(); 
    jeminloadert();
    pros::delay(1400);


    setintakespd(0);
    setintake2spd(0);
    pidTurnRel(-45, 1, 2, 100);
    pidMoveold(10, 0.5, 1, 60);
    jeminwingt();
    pidTurnRel(35, 1, 1);
    pidMoveold(-24, 1, 3, 100);
    //*/



    //RIGHT 3+4
    /*imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27.8, 0.5, 90, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 182, 0.8, 100);
    moveForSec(50, true, 0.9);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 185, 1.5, 100);
    jeminmech.set_value(true);
    skillscore();

    moveForSec(50, false, 1.3);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(15, 1, 1, 100); //13
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(30, 310, 3, 70, 1, 25, 5);
    pidGyro(22, 310, 2, 80, 1, 25, 10);
    setintake2spd(0);

    jemintaket();
    setintakespd(100);
    moveForSec(30, true, 0.7);
    setintakespd(0);
    //jemintaket();

    jeminwingt();

    pidGyro(-22, 270, 3, 70, 1, 25, 5, 1000);
    pidGyro(-13, 10, 1.5, 70, 1, 25, 2, 500);

    jeminwingt();
    jemintaket();
    pros::delay(200);
    moveForSec(70, true, 0.25);
    pidTurnRel(-40, 1, 1, 100);
    //*/


    //LEFT 3+4
    /*imu.set_heading(90);
    sort_on = false;
    storing = false;
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-5);

    pidwallGyro(27, 90, 270, 2, 80);
    jeminloadert();
    pidTurnAbs(180, 2, 0.7);

    pidGyro(10, 178, 0.8, 100);
    moveForSec(80, true, 0.6);
    
    moveForSec(50, false, 0.6);

    pidGyro(-19, 175, 1, 100);
    jeminmech.set_value(true);

    skillscore();
    jeminloadert();
    moveForSec(50, false, 1.5);
    
    setintake2spd(0);
    setintakespddiff(-100, -100);

    pidmove(13, 0.5, 1, 100);
    pidGyro(41, 52, 2, 80, 1, 25, 15);
    jeminloadert();
    moveForSec(30, true, 0.4);
    pidTurnAbs(220, 1, 0.8, 80);
    moveForSec(60, false, 0.6);

    setintakespd(-100);
    setintake2spd(40);
    pros::delay(1000);
    setintake2spd(0);
    setintakespd(0);
    jeminloadert();
    moveForSec(50, true, 0.2);


    pidGyro(27, 225, 2, 80, 1, 25, 5, 1000);
    pidGyro(6, 167, 1.5, 70, 1, 25, 1, 100);

    jeminwingt();
    pros::delay(200);
    moveForSec(70, false, 0.25);
    pidTurnRel(40, 1, 1, 100);
    //*/


void SAWP()
{
    imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();
    jeminwingt();

    setintakespd(-100);
    setintake2spd(-5);

    // storing = true;
    storing = true;
    moveForSec(30, true, 0.2);
    pidwallGyro(27, 270, 90, 2.5, 80);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);
    //pros::delay(200);

    // setintakespd(-100);
    // setintake2spd(-100);
    pidGyro(10, 180, 0.8, 100);
    moveForSec(60, true, 0.71);

    
    pidGyro(-38, 188, 2, 100);
    // moveForSec(70, false, 0.1);
    jeminmech.set_value(true);
    storing = false;
    // score_on = true;
    setintake2spd(-100);
    setintakespd(-100);

    moveForSec(50, false, 0.7);
    jeminloadert();
    //moveForSec(50, false, 0.7);
    score_on = false;
    setintakespd(-100);
    setintake2spd(0);



    pidGyro(33, 310, 2, 100, 1, 25, 5, 200);
    //pidmove(11, 1, 0.5, 100);

    storing = true;
    jeminmech.set_value(false);
    

    //pros::delay(1000);
    pidGyro(72, 270, 2, 100, 1, 20, 3);
    jeminloadert();

    // pidmove(48, 0.5, 3, 100);
    pidGyro(50, 210, 2, 70, 1, 25, 1);
    pros::delay(1);

    pidFrontWallGyro(21, 230, 270, 0.8, 100);


    pidTurnAbs(180, 3, 0.6);


    pidGyro(-20, 180, 1, 100);
    jeminmech.set_value(true);
    setintake2spd(-100);
    setintakespd(-100);
    moveForSec(60, false, 0.6);
    setintake2spd(0);

    pidGyro(34, 176, 1, 100);
    moveForSec(50, true, 1);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    pidmove(-17, 1, 0.5, 100);

    storing = true;
    jeminmech.set_value(false);
    pidGyro(-47, 230, 2, 100, 10);
    jeminmech.set_value(true);
    moveForSec(40, false, 0.3);
    storing = false;
    setintakespddiff(-100, -40);
    setintake2spd(60);
    moveForSec(30, false, 0.5);
}

    // storing = false;
    // jeminmech.set_value(true);
    // moveForSec(100, false, 0.5);
    // score_on = true; 
    // moveForSec(40, false, 0.3); 
    //*/



    //MIDDLE SOLO AWP
    /*imu.set_heading(270);
    sort_on = false;
    storing = false;
    jemintaket();

    setintakespd(-100);
    setintake2spd(0);
    pidWallMove(27.8, 0.5, 90, 2.5);
    jeminloadert();
    pidTurnAbs(180, 10, 0.8);

    pidGyro(10, 182, 0.8, 100);
    moveForSec(50, true, 0.9);
    
    //moveForSec(100, false, 0.35);

    pidGyro(-34, 185, 1.5, 100);
    jeminmech.set_value(true);
    setintakespd(-100);
    setintake2spd(-100);

    moveForSec(50, false, 1.6);
    jeminloadert();
    setintakespd(-100);
    setintake2spd(0);

    
    setintake2spd(-100);
    setintakespddiff(-100, 0);

    pidMoveold(13, 1, 1, 100);
    // pidTurnRel(96, 1, 1.2, 100);
    pidGyro(30, 310, 3, 70, 1, 25, 5);
    pidGyro(22, 310, 2, 80, 1, 25, 10);
    setintake2spd(0);

    jemintaket();
    setintakespd(70);
    moveForSec(30, true, 1.2);
    jemintaket();

    setintakespd(-100);
    pidmove(-23, 1, 2);
    pidGyro(46, 267, 3, 70, 5, 25, 1, 15);
    jeminloadert();
    moveForSec(30, true, 0.2);
    pidGyro(-23, 225, 2);

    //setintakespd(-100);
    setintake2spd(50);
    moveForSec(30, false, 0.3);
    //*/
