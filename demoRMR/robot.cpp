#include "robot.h"
#include <iostream>
#include <math.h>
#include <QDateTime>

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
    this->navigation = true;
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

    float lastFWspeed = forwardspeed;
    float lastRotSpeed = rotationspeed;

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

    double P_v = 500;
    double P_w = 5;

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
    if (l_error > 0.25 && pom_v < 40) {
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

    Pose p;
    p.x = x;
    p.y = y;
    p.fi = fi;
    p.timestamp = robotdata.timestamp;

    poseHistory.push_back(p);

    if (poseHistory.size() > 500) {
        poseHistory.erase(poseHistory.begin());
    }
}

double interpolate(double p0, double p1, double p2, double p3, double t) {
    return 0.5 * ((2 * p1) +
                  (-p0 + p2) * t +
                  (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                  (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
}

void robot::uloha_3(const std::vector<LaserData>& laserData)
{
    if (poseHistory.empty()) return;

    for (int i = 0; i < (int)laserData.size(); i++)
    {
        float dist_Li = laserData.at(i).scanDistance / 1000.0f;
        if (dist_Li < 0.2f || dist_Li > 3.5f) continue;
        if (dist_Li < 0.7 && dist_Li > 0.6) continue;

        uint32_t scanTime = laserData.at(i).timestamp;
        float xk, yk, fik;
        bool found = false;
        int idx = 1;

        if (poseHistory.size() >= 4) {
            for (idx = 1; idx < (int)poseHistory.size() - 2; idx++) {
                if (poseHistory[idx+1].timestamp > scanTime) {
                    found = true;
                    break;
                }
            }
        }

        if (found) {
            double t = (double)(scanTime - poseHistory[idx].timestamp) /
                       (double)(poseHistory[idx+1].timestamp - poseHistory[idx].timestamp);

            xk = interpolate(poseHistory[idx-1].x, poseHistory[idx].x, poseHistory[idx+1].x, poseHistory[idx+2].x, t);
            yk = interpolate(poseHistory[idx-1].y, poseHistory[idx].y, poseHistory[idx+1].y, poseHistory[idx+2].y, t);

            float diff = poseHistory[idx+1].fi - poseHistory[idx].fi;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            fik = poseHistory[idx].fi + t * diff;
        } else {
            xk = poseHistory.back().x;
            yk = poseHistory.back().y;
            fik = poseHistory.back().fi;
        }

        float angle_rad = (laserData.at(i).scanAngle / 360.0) * (2 * M_PI);

        float tx = xk + dist_Li * std::cos(fik - angle_rad);
        float ty = yk + dist_Li * std::sin(fik - angle_rad);

        int gridi = std::floor(tx / 0.05) + 140;
        int gridj = std::floor(ty / 0.05) + 140;

        if (gridi >= 0 && gridi < 280 && gridj >= 0 && gridj < 280) {
            if(map_temp[gridi][gridj] < 15) map_temp[gridi][gridj]++;
            if(map_temp[gridi][gridj] > 8) map[gridi][gridj] = 1;
        }
    }

    mapRC++;
    if (mapRC >= 100) {
        for(int i=0; i<280; i++) {
            for(int j=0; j<280; j++) {
                map_temp[i][j] = 0;
            }
        }
        mapRC = 0;
        std::cout << "Temp map reset" << std::endl;
    }



    // std::cout << "Pocet bodov v skene: " << laserData.size() << std::endl;
    // for (int i = 0; i < 5; i++) {
    //     printf("Bod [%d]: %.3f x , %.3f y\n", i, dist_x[i], dist_y[i]);

    // }

    vykresliMapu();
}

void robot::vykresliMapu() {
    QImage obr(280, 280, QImage::Format_RGB32);
    obr.fill(Qt::white);
    for(int i=0; i<280; i++) {
        for(int j=0; j<280; j++) {
            if(map[i][j] == 1) {
                obr.setPixel(i, 279 - j, qRgb(0, 0, 0));
            }
        }
    }
    emit publishMap(obr);
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



    uloha_3(copyOfLaserData);

    copyOfLaserData=laserData;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    processNavigation(laserData);


    emit publishLidar(copyOfLaserData, bHistogramVFH);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

int robot::processNavigation(const std::vector<LaserData> &xlaserData){

    double deltaXGlobal = goalXGlobal - x;
    double deltaYGlobal = goalYGlobal - y;


    double l_error = std::sqrt(deltaXGlobal*deltaXGlobal + deltaYGlobal*deltaYGlobal);
    if (l_error < 0.25 && navigation) {
        goalX = goalXGlobal;
        goalY = goalYGlobal;
        navigation = false;
        std::cout << "V cieli!!" << std::endl;
    }

    // treba nechat na zaciatku, spracuva laserData do Histogramu
    std::vector<LaserData> laserData;

    for(int i = 0; i < xlaserData.size(); i++){
        LaserData l = xlaserData.at(i);

        LaserData nl;
        nl.scanQuality = l.scanQuality;
        nl.scanAngle = 360.0f - l.scanAngle;
        nl.scanDistance = l.scanDistance;
        nl.timestamp = l.timestamp;

        laserData.insert(laserData.end(), nl);
    }


    this->processHistogram(laserData);


    this->calculateCandidatesNav();


    if(this->candidates.empty()){
        return 0;
    }

    int k = 0;
    float g_min = 100.0f;


    // smer cieľov bez uhľa
    double w_targetGlobal = std::atan2(deltaYGlobal, deltaXGlobal);

    double deltaX = goalX - x;
    double deltaY = goalY - y;

    double w_target = std::atan2(deltaY, deltaX);


    for(int i = 0; i < this->candidates.size(); i++){
        double wC = candidates.at(i);

        double g = 1 * getMinAngle(wC, w_targetGlobal) + 0.5 * getMinAngle(wC, w_target) + 1.1 * getMinAngle(wC, fi);

        if(g < g_min){
            g_min = g;
            k = i;
        }
    }

    if(this->navigation){

        this->goalX = this->candidatesX.at(k);
        this->goalY = this->candidatesY.at(k);
    }

    return 0;
}

void robot::processHistogram(const std::vector<LaserData> &laserData){
    for(int i = 0; i < nSector; i++){
        histogramVFH[i] = 0.0f;
    }


    // vytvorenie histogramu float
    for(int i = 0; i < laserData.size(); i++){
        if(laserData.at(i).scanDistance > VFHmin && laserData.at(i).scanDistance < VFHmax){
            // podla scanAngle priradime do spravnej stlpca

            // zistíme veľkost bodu v uhloch
            float dst = laserData.at(i).scanDistance;
            float alpha = asin(VFHpointSize / dst) / 3.14159 * 180;

            int from = ((laserData.at(i).scanAngle) - alpha) / sectorSize;
            int to = ((laserData.at(i).scanAngle) + alpha) / sectorSize + 1;


            for(int j = from; j < to; j++){
                histogramVFH[(j + nSector) % nSector] += 1 - (dst / VFHmax);
            }
        }
    }

    // vytvorenie binarneho histogramu
    if(bHistogramVFH.size() < nSector - 1){
        bHistogramVFH.erase(bHistogramVFH.begin(), bHistogramVFH.end());
        for(int i = 0; i < nSector; i++){
            bHistogramVFH.insert(bHistogramVFH.end(), histogramVFH[i] > VFHcutOffHigh);
        }
    }
    for(int i = 0; i < nSector; i++){
        if(histogramVFH[i] > VFHcutOffHigh)
            bHistogramVFH.at(i) = true;
        else if(histogramVFH[i] < VFHcutOffLow)
            bHistogramVFH.at(i) = false;
    }


    for(int i = laserData.size() * 2; i < laserData.size(); i++){
        if(laserData.at(i).scanDistance < VFHmaskMax && laserData.at(i).scanDistance > VFHpointSize / 2){
            float dst = laserData.at(i).scanDistance;

            if(dst < this->VFHpointSize * 3){
                for(int j = 0; j < nSector/2; j++){
                    bHistogramVFH.at(j) = true;
                }
            }
        }
    }

    for(int i = 0; i < laserData.size()/ 3; i++){
        if(laserData.at(i).scanDistance < VFHmaskMax && laserData.at(i).scanDistance > VFHpointSize / 2){
            float dst = laserData.at(i).scanDistance;
            if(dst < this->VFHpointSize * 2){
                for(int j = nSector/2; j < nSector; j++){
                    bHistogramVFH.at(j) = true;
                }

            }
        }
    }
}

void robot::calculateCandidatesNav(){
    // process bVFH diagram and calculate candidates

    this->candidates.erase(candidates.begin(), candidates.end());

    this->candidatesX.erase(candidatesX.begin(), candidatesX.end());
    this->candidatesY.erase(candidatesY.begin(), candidatesY.end());


    double deltaXGlobal = goalXGlobal - x;
    double deltaYGlobal = goalYGlobal - y;

    double w_targetGlobal = std::atan2(deltaYGlobal, deltaXGlobal);

    double w_errorGlobal = w_targetGlobal - fi;

    int sectorGlobalGoal = (int)(w_errorGlobal / sectorSizeRad + nSector) %  nSector;

    if(!bHistogramVFH.at(sectorGlobalGoal)){
        this->candidates.insert(this->candidates.end(), w_targetGlobal);

        this->candidatesX.insert(this->candidatesX.end(), x + cos(-w_targetGlobal));
        this->candidatesY.insert(this->candidatesY.end(), y - sin(-w_targetGlobal));
    }


    short sectorStart, sectorSize;
    bool sector = false;

    // todo prejst histogram, najst volne miesta
    for(int i = 0; i < nSector * 2; i++){
        if(!this->bHistogramVFH.at(i % nSector)){ // volne
            if(sector){
                sectorSize++;
            }else{
                sector = true;
                sectorStart = i;
                sectorSize = 1;
            }
        }else { // obsadene
            if(sector){
                if(sectorStart == 0){
                    sector = false;
                    continue; // z dovodu, ze ak je to prvy, nevieme povedať, či na -hodnotach nemá žiadne volne
                }
                if(sectorSize > 3){

                    float error1 = (sectorStart + 1.0f ) * this->sectorSizeRad + fi; // bod v histograme
                    float error2 = (sectorStart + sectorSize - 1.0f) * this->sectorSizeRad + fi; // bod v histograme

                    this->candidates.insert(this->candidates.end(), error1);
                    this->candidates.insert(this->candidates.end(), error2);

                    this->candidatesX.insert(this->candidatesX.end(), x + cos(-error1));
                    this->candidatesY.insert(this->candidatesY.end(), y - sin(-error1));

                    this->candidatesX.insert(this->candidatesX.end(), x + cos(-error2));
                    this->candidatesY.insert(this->candidatesY.end(), y - sin(-error2));

                }else {
                    // 1 bod v strede
                    float error = (sectorStart + sectorSize / 2.0f) * this->sectorSizeRad + fi; // bod v histograme

                    this->candidates.insert(this->candidates.end(), error);

                    this->candidatesX.insert(this->candidatesX.end(), x + cos(-error));
                    this->candidatesY.insert(this->candidatesY.end(), y - sin(-error));
                }
                sector = false;
            }
            if(i >= nSector){
                break;
            }
        }
    }

    for (int i = 0; i < candidates.size(); i++){
        if(candidates.at(i) < 0){
            candidates.at(i) += M_PI * 2;
        }
        if(candidates.at(i) > 2* M_PI){
            candidates.at(i) -= M_PI * 2;
        }
    }

}

double robot::getMinAngle(double a1i, double a2i){
    double a1 = a1i;
    double a2 = a2i;


    double diff = fmod(a1 - a2, 2 * M_PI);
    diff = fabs(diff);
    if (diff > M_PI)
        diff = 2 * M_PI - diff;
    return diff;
}

int robot::getGoalX(){
    int result = -((this->goalX - x) * cos(fi) + (this->goalY - y) * sin(fi)) * 100;

    return result;
}

int robot::getGoalY(){
    int result = -((this->goalY - y) * cos(fi) - (this->goalX - x) * sin(fi))  * 100;

    return result;
}


std::vector<int> robot::getCandidatesX(){
    std::vector<int> result;

    for (int i = 0; i < candidatesX.size(); i++) {
        result.insert(result.end(), -((this->candidatesX.at(i) - x) * cos(fi) + (this->candidatesY.at(i) - y) * sin(fi)) * 100);
    }

    return result;
}

std::vector<int> robot::getCandidatesY(){
    std::vector<int> result;
    for (int i = 0; i < candidatesY.size(); i++) {
        result.insert(result.end(), -((this->candidatesY.at(i) - y) * cos(fi) - (this->candidatesX.at(i) - x) * sin(fi))  * 100);
    }
    return result;
}


int robot::getGoalGlobalX(){
    int result = -((this->goalXGlobal - x) * cos(fi) + (this->goalYGlobal - y) * sin(fi))  * 100;

    return result;
}

int robot::getGoalGlobalY(){
    int result = -((this->goalYGlobal - y) * cos(fi) - (this->goalXGlobal - x) * sin(fi))  * 100;

    return result;
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
