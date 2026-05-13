#include "robot.h"
#include <iostream>
#include <math.h>
#include <QDateTime>
#include <fstream>
#include <sstream>

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

void robot::setLocation(double robotsetX, double robotsetY, double robotsetfi){
    this->x = robotsetX;
    this->y = robotsetY;
    this->fi = robotsetfi;
    this->fi_prev = fi_now-robotsetfi;
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
        fi_offset = ((robotdata.GyroAngle / 100.0) / 360.0)
            * (2 * M_PI);

        fi_prev = 0;
        fi = 0;
        return;
    }

    // vypočet natočenia ???
    double gyro_angle =
        ((robotdata.GyroAngle / 100.0) / 360.0)
        * (2 * M_PI);

    // absolútny heading od štartu
    fi_now = gyro_angle - fi_offset;

    // normalizácia
    while (fi_now > M_PI)
        fi_now -= 2 * M_PI;

    while (fi_now < -M_PI)
        fi_now += 2 * M_PI;

    // zmena uhla
    double delta_fi = fi_now - fi_prev;

    while (delta_fi > M_PI)
        delta_fi -= 2 * M_PI;

    while (delta_fi < -M_PI)
        delta_fi += 2 * M_PI;

    // ABSOLÚTNY uhol robota
    fi = fi_now;


    // rozdiel v každej vzorke
    short deltaLeft = (short)(robotdata.EncoderLeft - prevEncoderLeft);
    short deltaRight = (short)(robotdata.EncoderRight - prevEncoderRight);
    // prepočet na metre
    double lengthLeft = deltaLeft * tickToMeter;
    double lengthRight = deltaRight * tickToMeter;


    double length = (lengthLeft + lengthRight) / 2.0;


    if(localizationEnabled)
    {
        uloha_5_pohyb(length, delta_fi);
    }


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

    /*if (l_error < 0.5 and l_error > 0.05) {
        aim_v = 50;
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

fi_prev = fi_now;

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
    if (!mappingEnabled || poseHistory.empty()) return;

    if (poseHistory.empty()) return;


    if (mappingEnabled) {
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


    if (DONE_MApping && !mappingFinishedTriggered) {
        exportMapToCSV("final_mapa.csv");
        mappingFinishedTriggered = true; // Zabezpečí, že sa to spustí iba raz
    }

}

void robot::exportMapToCSV(const std::string& filename)
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        for (int j = 279; j >= 0; j--)
        {
            for (int i = 0; i < 280; i++)
            {
                file << (int)map[i][j];
                if (i < 279) file << ",";
            }
            file << "\n";
        }
        file.close();
        std::cout << "Mapa bola uspesne exportovana do: " << filename << std::endl;
    }
    else {
        std::cerr << "Nepodarilo sa otvorit subor na zapis" << std::endl;
    }
}

void robot::vykresliMapu() {
    QImage obr(280, 280, QImage::Format_RGB32);
    obr.fill(Qt::white);

    // 1. Vykreslenie prekážok (Steny - Čierna)
    for(int i=0; i<280; i++) {
        for(int j=0; j<280; j++) {
            if(map[i][j] == 1) {
                obr.setPixel(i, 279 - j, qRgb(0, 0, 0));
            }
        }
    }

    // 2. Vykreslenie ČASTÍC (Monte Carlo - Modrá)
    for (const auto& p : particles) {
        int pi = std::floor(p.x / 0.05) + 140;
        int pj = std::floor(p.y / 0.05) + 140;

        if (pi >= 0 && pi < 280 && pj >= 0 && pj < 280) {
            // Vykreslíme časticu ako jeden modrý pixel
            obr.setPixel(pi, 279 - pj, qRgb(0, 0, 255));
        }
    }

    // 3. Vykreslenie ODHADOVANEJ POLOHY (Zelený krížik)
    // Tieto hodnoty si vypočítal na konci funkcie uloha_5
    int ex = std::floor(estimatedX / 0.05) + 140;
    int ey = std::floor(estimatedY / 0.05) + 140;

    if (ex >= 0 && ex < 280 && ey >= 0 && ey < 280) {
        for(int dx = -1; dx <= 1; dx++) {
            for(int dy = -1; dy <= 1; dy++) {
                int px = ex + dx;
                int py = 279 - (ey + dy);
                if(px >= 0 && px < 280 && py >= 0 && py < 280)
                    obr.setPixel(px, py, qRgb(0, 255, 0));
            }
        }
    }

    // 4. Vykreslenie CIELA (Pôvodný kód - Červená)
    int gi = std::floor(goalXGlobal / 0.05) + 140;
    int gj = std::floor(goalYGlobal / 0.05) + 140;

    if (gi >= 0 && gi < 280 && gj >= 0 && gj < 280) {
        for(int x_off = -1; x_off <= 1; x_off++) {
            for(int y_off = -1; y_off <= 1; y_off++) {
                int px = gi + x_off;
                int py = 279 - (gj + y_off);
                if(px >= 0 && px < 280 && py >= 0 && py < 280)
                    obr.setPixel(px, py, qRgb(255, 0, 0));
            }
        }
    }

    emit publishMap(obr);
}

void robot::importMapFromCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Nepodarilo sa otvorit subor mapy na citanie: " << filename << std::endl;
        return;
    }

    // Vyčistíme aktuálnu mapu pred načítaním
    for(int i=0; i<280; i++) {
        for(int j=0; j<280; j++) {
            map[i][j] = 0;
            map_temp[i][j] = 0;
        }
    }

    std::string line;
    int j = 279; // Začíname od horného riadku (tak ako si to exportoval)

    while (std::getline(file, line) && j >= 0) {
        std::stringstream ss(line);
        std::string cell;
        int i = 0;

        while (std::getline(ss, cell, ',') && i < 280) {
            map[i][j] = std::stoi(cell);
            i++;
        }
        j--;
    }

    file.close();
    std::cout << "Mapa bola nacitana z: " << filename << std::endl;

    vykresliMapu();
}

void robot::uloha_4() {

    // 1. VYPOCET FLOOD FILL
    if (!cesta_vypocitana) {

        for(int i=0; i<280; i++) for(int j=0; j<280; j++) map_4[i][j] = 0;

        // zhruby stenu o 4 bloky

        int n = 4;
        for(int i=0; i<280; i++) {
            for(int j=0; j<280; j++) {
                if(map[i][j] == 1) {
                    for(int xx = -n; xx <= n; xx++) {
                        for(int yy = -n; yy <= n; yy++) {
                            int ni = i + xx;
                            int nj = j + yy;
                            if(ni>=0 && ni<280 && nj>=0 && nj<280) map_4[ni][nj] = 1;
                        }
                    }
                }
            }
        }

        int robot_i = std::floor(x / 0.05) + 140;
        int robot_j = std::floor(y / 0.05) + 140;
        int target_i = std::floor(goalXGlobal / 0.05) + 140;
        int target_j = std::floor(goalYGlobal / 0.05) + 140;

        if (target_i < 0 || target_i >= 280 || target_j < 0 || target_j >= 280 ||
            robot_i < 0 || robot_i >= 280 || robot_j < 0 || robot_j >= 280) {
            std::cout << "Robot alebo ciel su mimo mapy" << std::endl;
            return;
        }

        for(int i = 0; i < 280; i++) {
            for(int j = 0; j < 280; j++) {
                if(map_4[i][j] == 1) map_nav[i][j] = 255;
                else map_nav[i][j] = 0;
            }
        }

        if(map_nav[target_i][target_j] != 255) {
            map_nav[target_i][target_j] = 2;
        } else {
            std::cout << "Ciel je v prekazke" << std::endl;
            return;
        }

        // WAVE SPREAD

        int aktualna_vlna = 2;
        bool zmena = true;
        bool cesta_najdena = false;

        while (zmena && !cesta_najdena) {
            zmena = false;
            for (int i = 0; i < 280; i++) {
                for (int j = 0; j < 280; j++) {
                    if (map_nav[i][j] == aktualna_vlna) {
                        int di[] = {0, 0, 1, -1};
                        int dj[] = {1, -1, 0, 0};
                        for (int k = 0; k < 4; k++) {
                            int ni = i + di[k];
                            int nj = j + dj[k];
                            if (ni >= 0 && ni < 280 && nj >= 0 && nj < 280) {
                                if (map_nav[ni][nj] == 0) {
                                    map_nav[ni][nj] = aktualna_vlna + 1;
                                    zmena = true;
                                    if (ni == robot_i && nj == robot_j) cesta_najdena = true;
                                }
                            }
                        }
                    }
                }
            }
            aktualna_vlna++;
            if (aktualna_vlna >= 254) break;
        }

        // 3. EXTRAKCIA CELEJ CESTY DO VEKTORA
        if (cesta_najdena) {
            std::cout << "Cesta najdena, ukladam body!" << std::endl;
            planovana_cesta.clear();

            int curr_i = robot_i;
            int curr_j = robot_j;

            while(map_nav[curr_i][curr_j] > 2) {
                int n_val = map_nav[curr_i][curr_j];
                int next_i = curr_i, next_j = curr_j;
                int di[] = {0, 0, 1, -1, 1, 1, -1, -1};
                int dj[] = {1, -1, 0, 0, 1, -1, 1, -1};

                for(int k=0; k<8; k++) {
                    int ni = curr_i + di[k];
                    int nj = curr_j + dj[k];
                    if(ni>=0 && ni<280 && nj>=0 && nj<280) {
                        if(map_nav[ni][nj] > 1 && map_nav[ni][nj] < n_val) {
                            n_val = map_nav[ni][nj];
                            next_i = ni;
                            next_j = nj;

                        }
                    }
                }
                curr_i = next_i;
                curr_j = next_j;

                double wayX = (curr_i - 140) * 0.05;
                double wayY = (curr_j - 140) * 0.05;

                planovana_cesta.push_back({wayX, wayY});
            }

            if (planovana_cesta.size() > 2) {
                std::vector<std::pair<double, double>> zjednodusena_cesta;

                zjednodusena_cesta.push_back(planovana_cesta[0]);

                for (size_t k = 1; k < planovana_cesta.size() - 1; k++) {
                    // x1, y1 last bod
                    double x1 = zjednodusena_cesta.back().first;
                    double y1 = zjednodusena_cesta.back().second;

                    // x2, y2 atm bod
                    double x2 = planovana_cesta[k].first;
                    double y2 = planovana_cesta[k].second;

                    // x3, y3 future bod
                    double x3 = planovana_cesta[k+1].first;
                    double y3 = planovana_cesta[k+1].second;

                    double plocha = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2);

                    // ak nie je 0 tak je tam zakruta
                    if (std::abs(plocha) > 1e-4) {
                        zjednodusena_cesta.push_back(planovana_cesta[k]);
                    }
                }

                // final point
                zjednodusena_cesta.push_back(planovana_cesta.back());

                // Prepiseme staru za novu
                planovana_cesta = zjednodusena_cesta;

                std::cout << "Cesta zjednodusena na " << planovana_cesta.size() << " bodov (zakrut)." << std::endl;
            }

            cesta_vypocitana = true;
        } else {
            std::cout << "Cestu do ciela sa nepodarilo najst!" << std::endl;
            cesta_vypocitana = true;
        }
    }

    //NAVIGACIA (nonstop)

    if (cesta_vypocitana && !planovana_cesta.empty()) {

        double aktualny_ciel_X = planovana_cesta.front().first;
        double aktualny_ciel_Y = planovana_cesta.front().second;

        this->goalX = aktualny_ciel_X;
        this->goalY = aktualny_ciel_Y;

        double vzdialenost = std::hypot(aktualny_ciel_X - x, aktualny_ciel_Y - y);

        if (vzdialenost < 0.10) {
            planovana_cesta.erase(planovana_cesta.begin());

            if (planovana_cesta.empty()) {
                std::cout << "finish ciel" << std::endl;
                cesta_vypocitana = false;
            }
        }
    }
}

void robot::EnlargeMap() {

    for(int i = 0; i < 280; i++) {
        for(int j = 0; j < 280; j++) {
            map_4[i][j] = 0;
        }
    }

    int n = 3;

    for(int i = 0; i < 280; i++) {
        for(int j = 0; j < 280; j++) {
            if(map[i][j] == 1) {
                for(int x_off = -n; x_off <= n; x_off++) {
                    for(int y_off = -n; y_off <= n; y_off++) {
                        int ni = i + x_off;
                        int nj = j + y_off;

                        if(ni >= 0 && ni < 280 && nj >= 0 && nj < 280) {
                            map_4[ni][nj] = 1;
                        }
                    }
                }
            }
        }
    }
    std::cout << "finish" << std::endl;
}


void robot::uloha_5(const std::vector<LaserData>& laserData)
{
    // =========================================================
    // 1. INITIALIZÁCIA PARTICLES
    // =========================================================

    if (particles.empty()) {

        static std::random_device rd;
        static std::default_random_engine gen(rd());

        std::uniform_real_distribution<double> distPos(-7.0, 7.0);
        std::uniform_real_distribution<double> distAngle(-M_PI, M_PI);

        for (int i = 0; i < numParticles; ++i) {

            Particle p;

            p.x = distPos(gen);
            p.y = distPos(gen);
            p.fi = distAngle(gen);
            p.weight = 1.0 / numParticles;

            particles.push_back(p);
        }
    }

    // =========================================================
    // 2. VÝPOČET VÁH
    // =========================================================

    // =========================================================
    // 2. VÝPOČET VÁH (Model priemernej chyby - MAE)
    // =========================================================

    double totalWeight = 0.0;

    for (auto& p : particles) {
        double sum_error = 0.0;
        int valid_rays = 0;

        for (int i = 0; i < (int)laserData.size(); i += 5) {
            double measured = laserData[i].scanDistance / 1000.0;

            // Tvoj filter prekážok a robota
            if (measured < 0.2 || measured > 3.5) continue;
            if (measured >= 0.6 && measured <= 0.7) continue;

            double angle_rad = (laserData[i].scanAngle / 360.0) * (2 * M_PI);
            double expected = expectedRangeFromMapBresenham(p.x, p.y, p.fi - angle_rad, 3.5);

            // Spočítame absolútnu odchýlku v metroch
            double diff = std::abs(measured - expected);
            sum_error += diff;
            valid_rays++;
        }

        // Výpočet priemernej chyby pre túto časticu
        double avg_error = (valid_rays > 0) ? (sum_error / valid_rays) : 10.0;

        // Exponenciálna premena: čím menšia chyba, tým obrovskejšia váha.
        // Konštanta 0.1 určuje prísnosť. Ak to nebude konvergovať, zníž ju na 0.05
        p.weight = std::exp(-avg_error / 0.1);

        // Ochrana pred nulou
        if (p.weight < 1e-9) p.weight = 1e-9;

        totalWeight += p.weight;
    }

    // =========================================================
    // 3. NORMALIZÁCIA
    // =========================================================

    if (totalWeight < 1e-12)
        totalWeight = 1e-12;

    for (auto& p : particles) {

        p.weight /= totalWeight;
    }

    // =========================================================
    // 4. ODHAD POLOHY
    // =========================================================

    double max_weight = -1.0;

    if(particles.empty()) return;
    Particle best_particle = particles[0];

    for (const auto& p : particles) {

        if (p.weight > max_weight) {

            max_weight = p.weight;
            best_particle = p;
        }
    }

    // Vyhladenie
    double alpha = 0.2;

    estimatedX =
        alpha * best_particle.x +
        (1.0 - alpha) * estimatedX;

    estimatedY =
        alpha * best_particle.y +
        (1.0 - alpha) * estimatedY;

    estimatedFi = best_particle.fi;

    // =========================================================
    // 5. EFFECTIVE SAMPLE SIZE
    // =========================================================

    double neff = 0.0;

    for (const auto& p : particles) {

        neff += p.weight * p.weight;
    }

    if(neff < 1e-12)
        neff = 1e-12;
    neff = 1.0 / neff;

    // =========================================================
    // 6. RESAMPLING
    // =========================================================

    // Resamplujeme iba ak treba
    if (true) {

        std::vector<Particle> newParticles;
        newParticles.reserve(numParticles);

        static std::default_random_engine gen_resample;
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::uniform_real_distribution<double> distPos(-7.0, 7.0);
        std::uniform_real_distribution<double> distAngle(-M_PI, M_PI);

        std::normal_distribution<double> jitter_pos(0.0, 0.02); // Mierne väčší jitter
        std::normal_distribution<double> jitter_ang(0.0, 0.05);

        // Definujeme, koľko častíc sa resampluje a koľko sa hodí náhodne
        int random_count = numParticles * 0.05; // 5% prieskumníkov (napr. 25 z 500)
        int resampled_count = numParticles - random_count;

        for (int i = 0; i < numParticles; ++i) {
            if (i < resampled_count) {
                // TVOJ RULETOVÝ VÝBER
                double r = dist(gen_resample);
                double cumulativeWeight = 0.0;
                for (const auto& p : particles) {
                    cumulativeWeight += p.weight;
                    if (r <= cumulativeWeight) {
                        Particle np = p;
                        np.x += jitter_pos(gen_resample);
                        np.y += jitter_pos(gen_resample);
                        np.fi += jitter_ang(gen_resample);

                        while (np.fi > M_PI) np.fi -= 2.0 * M_PI;
                        while (np.fi < -M_PI) np.fi += 2.0 * M_PI;

                        np.weight = 1.0 / numParticles;
                        newParticles.push_back(np);
                        break;
                    }
                }
            } else {
                // TVOJI PRIESKUMNÍCI (Ochrana proti lokálnemu minimu)
                Particle np;
                np.x = distPos(gen_resample);
                np.y = distPos(gen_resample);
                np.fi = distAngle(gen_resample);
                np.weight = 1.0 / numParticles;
                newParticles.push_back(np);
            }
        }

        while (newParticles.size() < numParticles) {
            newParticles.push_back(particles.back());
        }
        particles = std::move(newParticles);
    }

    // =========================================================
    // 7. VIZUALIZÁCIA
    // =========================================================

    std::vector<std::pair<int,int>> particleCells;

    for (const auto& p : particles) {

        int pi = std::floor(p.x / 0.05) + 140;
        int pj = std::floor(p.y / 0.05) + 140;

        if (pi >= 0 && pi < 280 &&
            pj >= 0 && pj < 280) {

            if (map[pi][pj] == 0) {

                map[pi][pj] = 2;

                particleCells.push_back({pi, pj});
            }
        }
    }

    vykresliMapu();

    // vyčistenie mapy
    for (auto const& cell : particleCells) {

        map[cell.first][cell.second] = 0;
    }
}

void robot::uloha_5_pohyb(double length, double delta_fi)
{
    if (particles.empty()) return;

    static std::random_device rd;
    static std::default_random_engine gen(rd());

    // MENŠÍ ALE NIE NULOVÝ ŠUM
    double trans_sigma = 0.01 + std::abs(length) * 0.01;

    double rot_sigma = 0.01 + std::abs(delta_fi) * 0.01;

    std::normal_distribution<double>noise_length(0.0, trans_sigma);

    std::normal_distribution<double>noise_angle(0.0, rot_sigma);

    for (auto& p : particles) {

        double noisy_length = length + noise_length(gen);

        double noisy_delta_fi = delta_fi + noise_angle(gen);

        // najprv rotácia
        p.fi += noisy_delta_fi;

        while (p.fi > M_PI)
            p.fi -= 2.0 * M_PI;

        while (p.fi < -M_PI)
            p.fi += 2.0 * M_PI;

        // potom translácia
        p.x += noisy_length * std::cos(p.fi);
        p.y += noisy_length * std::sin(p.fi);
    }
}


double robot::expectedRangeFromMapBresenham(double x, double y, double angle, double maxRange)
{
    // koncový bod lúča
    double endX = x + maxRange * std::cos(angle);
    double endY = y + maxRange * std::sin(angle);

    int x0 = std::floor(x / cellSize) + originI;
    int y0 = std::floor(y / cellSize) + originJ;
    int x1 = std::floor(endX / cellSize) + originI;
    int y1 = std::floor(endY / cellSize) + originJ;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    int xcur = x0;
    int ycur = y0;

    while (true) {
        // mimo mapy
        if (xcur < 0 || xcur >= mapWidth || ycur < 0 || ycur >= mapHeight)
            break;

        // narazil na stenu
        if (map[xcur][ycur] == 1) {
            double wx = (xcur - originI) * cellSize;
            double wy = (ycur - originJ) * cellSize;
            return std::sqrt((wx - x)*(wx - x) + (wy - y)*(wy - y));
        }

        if (xcur == x1 && ycur == y1)
            break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; xcur += sx; }
        if (e2 <  dx) { err += dx; ycur += sy; }
    }

    return maxRange;
}

void robot::useMonteCarloPose()
{
    x = estimatedX;
    y = estimatedY;
    fi = estimatedFi;

    fi_prev = fi;

    std::cout << "Pose updated from Monte Carlo:"
              << " X=" << x
              << " Y=" << y
              << " FI=" << fi
              << std::endl;
}

void robot::setAutoMode(bool state)
{
    useDirectCommands = !state;
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

    if(localizationEnabled)
    {
        uloha_5(laserData);
    }



    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    if (rezim_navigacie == 1) {
        processNavigation(laserData); // Úloha 2
    }
    else if (rezim_navigacie == 2) {
        uloha_4(); // Úloha 4
    }


    emit publishLidar(copyOfLaserData, bHistogramVFH);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

int robot::processNavigation(const std::vector<LaserData> &laserData){
    // treba nechat na zaciatku, spracuva laserData do Histogramu
    this->processHistogram(laserData);


    /*double deltaXGlobal = goalXGlobal - x;
    double deltaYGlobal = goalYGlobal - y;

    double w_targetGlobal = std::atan2(deltaYGlobal, deltaXGlobal);

    double w_errorGlobal = 360 - (w_targetGlobal - fi) / M_PI * 180;

    int sectorGlobalGoal = w_errorGlobal / nSector + nSector;
    sectorGlobalGoal %= nSector;

    // iba v prípade, že nie je dosť blizko
    double deltax = this->goalXGlobal - x;
    double deltay = this->goalYGlobal - y;

    double l_error_global = std::sqrt(deltax*deltax + deltay*deltay);

    if(l_error_global < 0.5){
        this->goalX = this->goalXGlobal;
        this->goalY = this->goalYGlobal;
        return 0;
    }

    double deltaX = goalX - x;
    double deltaY = goalY - y;
    double l_error = std::sqrt(deltaX*deltaX + deltaY*deltaY);


    if(goalX != goalXGlobal || goalY != goalYGlobal){

        if(l_error > 0.7){
            return 0;
        }
    }


    // ak sektor ktory je smerom global cielu je volny, tak nastavime ciel na global ciel
    // vypocitame smer k global cielu a porovname
    // poloha - natocenie
    bool emptyGG = 1;

    for(int j = sectorGlobalGoal - 2; j <= sectorGlobalGoal + 2; j++){
        int k = j < 0 ? j + nSector : j >= nSector ? j - nSector : j;
        emptyGG &= !bHistogramVFH.at(k);
    }

    if(emptyGG){
        this->goalX = this->goalXGlobal;
        this->goalY = this->goalYGlobal;

        return 0;
    }

    // pocitame s tym, ze ak nie su predne smery volne, tak menime smer - iba v pripade, že nie je nastaveny glabal goal
    if((bHistogramVFH.at(0) || bHistogramVFH.at(nSector - 1) || bHistogramVFH.at(1) || bHistogramVFH.at(nSector - 2)) || l_error < 0.2){
        int goalSector = -1;

        // todo dat podmienku, aby nechcel menit smer az moc casto
        for(int i = 1; i < 5; i++){
            if(!bHistogramVFH.at((sectorGlobalGoal + i + nSector) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector + 1) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector - 1) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector + 2) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector - 2) % nSector)){

                goalSector = (sectorGlobalGoal + i + nSector) % nSector;
                break;
            }else if(!bHistogramVFH.at((sectorGlobalGoal - i + nSector) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector + 1) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector - 1) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector + 2) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector - 2) % nSector)){

                goalSector = (sectorGlobalGoal - i + nSector) % nSector;
                break;
            }
        }

        if(goalSector != -1){

            float w_error = -fi + goalSector * sectorSize / 180 * M_PI;

            this->goalX = x + cos(w_error);
            this->goalY = y - sin(w_error);

            std::cout << "w error: " << w_error << std::endl;
            std::cout << "sector ciel: " << goalSector << std::endl;
            std::cout << "Nastavujem ciel na: " << this->goalX << ", "  << this->goalY << std::endl;
            return 0;
        }


        // todo dat podmienku, aby nechcel menit smer az moc casto
        for(int i = 1; i < 5; i++){
            if(!bHistogramVFH.at((sectorGlobalGoal + i + nSector) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector + 1) % nSector) &&
                !bHistogramVFH.at((sectorGlobalGoal + i + nSector - 1) % nSector)){

                goalSector = (sectorGlobalGoal + i + nSector) % nSector;
                break;
            }else if(!bHistogramVFH.at((sectorGlobalGoal - i + nSector) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector + 1) % nSector) &&
                       !bHistogramVFH.at((sectorGlobalGoal - i + nSector - 1) % nSector)){

                goalSector = (sectorGlobalGoal - i + nSector) % nSector;
                break;
            }
        }

        if(goalSector != -1){

            float w_error = -fi + goalSector * sectorSize / 180 * M_PI;

            this->goalX = x + cos(w_error);
            this->goalY = y - sin(w_error);

            std::cout << "w error: " << w_error << std::endl;
            std::cout << "sector ciel: " << goalSector << std::endl;
            std::cout << "Nastavujem ciel na: " << this->goalX << ", "  << this->goalY << std::endl;
            return 0;
        }



        return 0;
    }
*/
    return 0;
}

void robot::processHistogram(const std::vector<LaserData> &laserData){
    for(int i = 0; i < nSector; i++){
        histogramVFH[i] = 0.0f;
    }

    bHistogramVFH.clear();
    bHistogramVFH.erase(bHistogramVFH.begin(), bHistogramVFH.end());

    // vytvorenie histogramu
    for(int i = 0; i < laserData.size(); i++){
        if(laserData.at(i).scanDistance > VFHmin && laserData.at(i).scanDistance < VFHmax){
            // podla scanAngle priradime do spravnej stlpca

            // zistíme veľkost bodu v uhloch
            float dst = laserData.at(i).scanDistance;
            double val = VFHpointSize / dst;

            val = std::clamp(val, -1.0, 1.0);

            float alpha = asin(val);

            int from = ((laserData.at(i).scanAngle) - alpha) / sectorSize;
            int to = ((laserData.at(i).scanAngle) + alpha) / sectorSize + 1;


            for(int j = from; j < to; j++){
                histogramVFH[(j + nSector) % nSector] += 1 - (dst / VFHmax);
            }
        }
    }


    for(int i = 0; i < nSector; i++){
        bHistogramVFH.insert(bHistogramVFH.end(), histogramVFH[i] > VFHcutOff);
    }
}


int robot::getGoalX(){
    int result = -((this->goalX - x) * cos(fi) + (this->goalY - y) * sin(fi))  * 100;

    return result;
}

int robot::getGoalY(){
    int result = -((this->goalY - y) * cos(fi) - (this->goalX - x) * sin(fi))  * 100;

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
