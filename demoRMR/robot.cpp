#include "robot.h"
#include <iostream>
#include <math.h>

robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
#endif
#ifndef DISABLE_SKELETON
qRegisterMetaType<skeleton>("skeleton");
#endif
}

void robot::initAndStartRobot(std::string ipaddress)
{

    forwardspeed=0;
    rotationspeed=0;
    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters([this](const std::vector<LaserData>& dat)->int{return processThisLidar(dat);},ipaddress);
    robotCom.setRobotParameters([this](const TKobukiData& dat)->int{return processThisRobot(dat);},ipaddress);
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters(std::bind(&robot::processThisCamera,this,std::placeholders::_1),"http://"+ipaddress+":8000/stream.mjpg");
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters(std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setGoal(double goalX, double goalY){
    this->goalXGlobal = goalX;
    this->goalYGlobal = goalY;
    this->goalX = goalX;
    this->goalY = goalY;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
        robotCom.setRotationSpeed(rots);
    else if(forw!=0 && rots==0)
        robotCom.setTranslationSpeed(forw);
    else if((forw!=0 && rots!=0))
        robotCom.setArcSpeed(forw,forw/rots);
    else
        robotCom.setTranslationSpeed(0);
    useDirectCommands=1;
}

void robot::uloha_1(const TKobukiData &robotdata){
    // LOKALIZACIA

    //prvy beh setne minuly krok na realny kedze encoder môže začinat nie z nuly
    if(isFirstRun){
        useDirectCommands = 0;
        goalX = 0;
        goalY = 0;
        x = 0;
        y = 0;
        prevEncoderLeft = robotdata.EncoderLeft;
        prevEncoderRight = robotdata.EncoderRight;
        isFirstRun = false;
        fi_prev = ((robotdata.GyroAngle/ 100.0)/360.0)*(2*M_PI);
        return;
    }

    // vypočet natočenia ???
    fi_now = ((robotdata.GyroAngle/ 100.0)/360.0)*(2*M_PI);

    fi = fi_now - fi_prev;
    while (fi > M_PI) fi -= 2 * M_PI;
    while (fi < -M_PI) fi += 2 * M_PI;
    // rozdiel v každej vzorke
    short deltaLeft = (short)(robotdata.EncoderLeft - prevEncoderLeft);
    short deltaRight = (short)(robotdata.EncoderRight - prevEncoderRight);

    // prepočet na metre
    double lengthLeft = deltaLeft * tickToMeter;
    double lengthRight = deltaRight * tickToMeter;

    double length = (lengthLeft + lengthRight) / 2.0;
    x += length * std::cos(fi);
    y += length * std::sin(fi);

    //prepisanie k+1
    prevEncoderLeft = robotdata.EncoderLeft;
    prevEncoderRight = robotdata.EncoderRight;


    // POLOHOVANIE ZDRUZENY REGULATOR

    double deltax = goalX - x;
    double deltay = goalY - y;

    //double sides = deltay/deltax;
    double w_target = std::atan2(deltay, deltax);

    double l_error = std::sqrt(deltax*deltax + deltay*deltay);
    double w_error = w_target - fi;
    while (w_error > M_PI) w_error -= 2 * M_PI;
    while (w_error < -M_PI) w_error += 2 * M_PI;

    double P_v = 200;
    double P_w = 5;

    double v_max = 250;
    double w_max = 0.3;

    double pom_v = P_v * l_error;
    double pom_w = P_w * w_error;

    // obmedzenie na max rychlosť otáčania
    if (pom_w > w_max) {
        pom_w = w_max;
    } else if (pom_w < - w_max){
        pom_w = -w_max;
    }

    // obmedzenie na max rychlosť
    if (pom_v > v_max) {
        pom_v = v_max;
    }

    // dzžanie rýchlosti pokial nepríde blízko
    if (l_error > 0.5 && pom_v < 40) {
        pom_v = 200;
    }

    // želane rýchlosti
    double aim_v = pom_v;
    double aim_w = pom_w;

    //ak je väčši uhol natočenia ako 0.5 tak sa najprv iba točí
    if (std::abs(w_error) > 0.5) {
        aim_v = 0;
    }

    // ak je blízko ciela tak zastane už
    if (l_error < 0.05) {
        aim_v = 0;
        aim_w = 0;
    }

    /*if (std::abs(w_error) < 0.15) {
        aim_w = 0;
    }*/

    if (l_error < 0.5 and l_error > 0.05) {
        aim_v = 50;
    }

    // rampa
    // výpočet odchýlky od vzorky akou ide rampa
    double diffV = aim_v - forwardspeed;
    double diffW = aim_w - rotationspeed;

    // ak je odchylka väčšia tak sa rýchlosť iba zväčší o vzorku
    if (std::abs(diffV) > maxAccV) {
        if (diffV > 0) {
            forwardspeed += maxAccV;
        } else {
            forwardspeed -= maxAccV;            // možno zmenit na forwardspeed = aim_v, keďže neviem či chceme spomalovať aj rampu
        }
    } else {
        forwardspeed = aim_v;
    }

    // nemusi byt minus rampa

    if (std::abs(diffW) > maxAccW) {
        if (diffW > 0) {
            rotationspeed += maxAccW;
        } else {
            rotationspeed -= maxAccW;           // možno zmenit na rotationspeed = aim_w, keďže neviem či chceme spomalovať aj rampu
        }
    } else {
        rotationspeed = aim_w;
    }

}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(const TKobukiData &robotdata)
{


    uloha_1(robotdata);





    ///tu mozete robit s datami z robota



///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(x,y,fi, forwardspeed, rotationspeed);
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(const std::vector<LaserData>& laserData)
{
    copyOfLaserData=laserData;




    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    processNavigation(laserData);


    emit publishLidar(copyOfLaserData, bHistogramVFH);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

int robot::processNavigation(const std::vector<LaserData> &laserData){
    for(int i = 0; i < nSector; i++){
        histogramVFH[i] = 0.0f;
    }

    // vytvorenie histogramu
    for(int i = 0; i < laserData.size(); i++){
        if(laserData.at(i).scanDistance > VFHmin && laserData.at(i).scanDistance < VFHmax){
            // podla scanAngle priradime do spravnej stlpca

            // zistíme veľkost bodu v uhloch
            float dst = laserData.at(i).scanDistance;
            float alpha = asin(VFHpointSize / dst) / 3.14159 * 180;

            int from = (laserData.at(i).scanAngle - alpha) / sectorSize;
            int to = (laserData.at(i).scanAngle + alpha) / sectorSize + 1;


            for(int j = from; j < to; j++){
                if(j < 0){
                    histogramVFH[j + nSector] += 1 - (dst / VFHmax);
                }else if (j >= nSector){
                    histogramVFH[j - nSector] += 1 - (dst / VFHmax);
                }else{
                    histogramVFH[j] += 1 - (dst / VFHmax);
                }
            }
        }
    }

    bHistogramVFH.clear();
    bHistogramVFH.erase(bHistogramVFH.begin(), bHistogramVFH.end());

    for(int i = 0; i < nSector; i++){
        bHistogramVFH.insert(bHistogramVFH.end(), histogramVFH[i] > VFHcutOff);
    }


    // mame histogram, polohu, ciel a natocenie
    // vytvarame ciastkovy smer


    for(int i = 0; i < nSector; i++){
        std::cout << bHistogramVFH.at(i) << ";";
    }
    std::cout << "end;" << std::endl;


    // todo - kontrola, ci nevieme dostat priamo na final ciel - uhly
    if(true){ // todo zmenit podminku
        this->goalX = this->goalXGlobal;
        this->goalY = this->goalYGlobal;
        return 0;
    }

    // pocitame s tym, ze ak su predne smery volne, tak nemusime menit smer
    if(bHistogramVFH.at(0) || bHistogramVFH.at(nSector - 1)){

        // todo - vyber kandidatskeho smeru
        // taky co najblizsie ide smerom k konecnemu cielu, ale zaroven nie je nepriechodny


        this->goalX = 0;
        this->goalY = 0;
        return 0;
    }

    return 0;
}

  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
